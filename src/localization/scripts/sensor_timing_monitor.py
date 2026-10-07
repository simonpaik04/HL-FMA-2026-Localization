#!/usr/bin/python3
"""Observe timing without restamping sensors or changing any system clock."""
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from timing_core import TimingWindow, check_host_clock, count, distribution, positive

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool


def status_message(name, level, message, values):
    status = DiagnosticStatus(name=name, level=level, message=message, hardware_id='pc_clock_and_sensor_headers')
    status.values = [KeyValue(key=str(key), value=json.dumps(value, ensure_ascii=False, allow_nan=False)
                             if not isinstance(value, str) else value) for key, value in sorted(values.items())]
    return status


class SensorTimingMonitor:
    def __init__(self):
        self.policy = rospy.get_param('~sensor_timing_monitor')
        self.clock_policy = rospy.get_param('~clock_preflight')
        self.sim = bool(rospy.get_param('/use_sim_time', False))
        self.gps_receipt_mode = bool(rospy.get_param('~gps_driver/use_ros_time', False))
        for key in ('publish_rate_hz', 'host_check_period_sec', 'host_check_stale_sec',
                    'driver_diagnostics_stale_sec', 'window_sec', 'clock_jump_tolerance_sec',
                    'future_tolerance_sec', 'max_header_age_p95_sec',
                    'max_gnss_receipt_difference_sec'):
            self.policy[key] = positive(self.policy[key], key)
        self.policy['max_samples'] = count(self.policy['max_samples'], 'max_samples', 2)
        self.policy['min_samples'] = count(self.policy['min_samples'], 'min_samples')
        if self.policy['min_samples'] > self.policy['max_samples']:
            raise ValueError('min_samples must not exceed max_samples')
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.host = {'ready': False, 'status': 'HOST_CLOCK_NOT_CHECKED', 'hardware_sensor_sync_verified': False}
        self.host_stamp = None
        self.driver_values = {}
        self.driver_level = DiagnosticStatus.STALE
        self.driver_stamp = None
        self.gnss_differences = deque(maxlen=int(self.policy['max_samples']))
        self.windows = {}
        for name, config in self.policy['sensors'].items():
            for key in ('expected_interval_sec', 'max_gap_sec', 'stale_sec'):
                config[key] = positive(config[key], name+'/'+key)
            self.windows[name] = TimingWindow(self.policy['window_sec'], self.policy['max_samples'],
                                              config['max_gap_sec'], self.policy['clock_jump_tolerance_sec'])
        self.diagnostics = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size=2)
        self.clock_ready = rospy.Publisher(rospy.get_param('~internal_topics/clock_ready'), Bool,
                                          queue_size=2, latch=False)
        topics = rospy.get_param('~topics')
        self.subscribers = [rospy.Subscriber(topics[key], cls, self._sensor_callback,
                                             callback_args=name, queue_size=200)
                            for name, key, cls in [('gps', 'gps_fix', NavSatFix), ('imu', 'imu_data', Imu),
                                                   ('encoder', 'encoder_twist', TwistWithCovarianceStamped)]]
        self.subscribers.append(rospy.Subscriber(rospy.get_param('~gps_driver_timing_topic'), DiagnosticArray,
                                                 self._driver_callback, queue_size=100))
        rospy.on_shutdown(self.stop.set)
        self.host_worker = threading.Thread(target=self._host_loop, daemon=True)
        self.publisher_worker = threading.Thread(target=self._publish_loop, daemon=True)
        self.clock_ready.publish(Bool(data=False))
        self.host_worker.start()
        self.publisher_worker.start()

    def _sensor_callback(self, message, name):
        receipt_mono = time.monotonic()
        receipt_ros = rospy.Time.now().to_sec()
        with self.lock:
            self.windows[name].add(message.header.stamp.to_sec(), receipt_ros, receipt_mono)

    def _driver_callback(self, message):
        received = time.monotonic()
        for status in message.status:
            if status.name != 'ublox_gps: measurement timing':
                continue
            values = {value.key: value.value for value in status.values}
            values['_monitor_offset_parse_valid'] = 'false'
            with self.lock:
                self.driver_values = values
                self.driver_level = status.level
                self.driver_stamp = received
                if values.get('utc_valid') == 'true':
                    try:
                        value = float(values['receipt_minus_utc_clock_offset_plus_transport_ns'])/1e9
                        if math.isfinite(value):
                            self.gnss_differences.append((received, value))
                            values['_monitor_offset_parse_valid'] = 'true'
                    except (KeyError, ValueError, OverflowError):
                        pass

    def _host_loop(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            try:
                result = check_host_clock(self.clock_policy, self.sim)
            except Exception as error:
                result = {'ready': False, 'status': 'HOST_CLOCK_CHECK_FAILED', 'reason': str(error),
                          'hardware_sensor_sync_verified': False}
            with self.lock:
                self.host, self.host_stamp = result, time.monotonic()
            self.stop.wait(self.policy['host_check_period_sec'])

    def _sensor_status(self, name, now):
        window = self.windows[name].snapshot(now)
        config = self.policy['sensors'][name]
        level, reasons = DiagnosticStatus.OK, []
        age = window['last_receipt_age_sec']
        if age is None or age > config['stale_sec']:
            level = DiagnosticStatus.ERROR
            reasons.append('NOT_RECEIVED' if age is None else 'STALE_RECEIPT')
        if window['invalid_count'] or window['backwards_count']:
            level = max(level, DiagnosticStatus.ERROR)
            reasons.append('INVALID_OR_BACKWARDS_STAMP')
        if window['age']['count'] < self.policy['min_samples']:
            level = max(level, DiagnosticStatus.WARN)
            reasons.append('WARMUP_OR_TOO_FEW_SAMPLES')
        if window['duplicate_count'] or window['gap_count'] or window['clock_jump_count']:
            level = max(level, DiagnosticStatus.WARN)
            reasons.append('DUPLICATE_GAP_OR_CLOCK_JUMP')
        if window['age']['count']:
            if (window['age']['p95_sec'] > self.policy['max_header_age_p95_sec'] or
                    window['age']['min_sec'] < -self.policy['future_tolerance_sec']):
                level = max(level, DiagnosticStatus.WARN)
                reasons.append('RECEIPT_HEADER_DIFFERENCE_OUTSIDE_LIMIT')
        semantics = config['stamp_semantics']
        if name == 'gps':
            actual = self.driver_values.get('stamp_source', 'driver_confirmation_unavailable')
            semantics += '; actual='+actual
        window.update(stamp_semantics=semantics, physical_measurement_sync_verified=False,
                      pre_stamp_device_latency_measured=False,
                      receipt_minus_header_scope='clock difference plus post-stamp path; never hardware latency alone',
                      configured_expected_interval_sec=config['expected_interval_sec'],
                      use_sim_time=self.sim)
        return status_message('sensor_timing/'+name, level, ', '.join(reasons) or 'HEADER_TIMING_OBSERVED', window)

    def _driver_status(self, now):
        while self.gnss_differences and self.gnss_differences[0][0] < now-self.policy['window_sec']:
            self.gnss_differences.popleft()
        values = dict(self.driver_values)
        age = None if self.driver_stamp is None else now-self.driver_stamp
        metrics = distribution(value for _, value in self.gnss_differences)
        values.update(diagnostic_receipt_age_sec=age, receipt_minus_utc_stats=metrics,
                      offset_scope='PC UTC clock offset + transport/callback delay, not transport latency alone',
                      configured_gps_receipt_mode=self.gps_receipt_mode,
                      hardware_sensor_sync_verified=False)
        if age is None or age > self.policy['driver_diagnostics_stale_sec']:
            return status_message('sensor_timing/gnss_utc', DiagnosticStatus.ERROR,
                                  'GNSS_TIMING_DIAGNOSTICS_MISSING_OR_STALE', values)
        level = self.driver_level
        reasons = []
        if values.get('utc_valid') != 'true':
            level = max(level, DiagnosticStatus.ERROR); reasons.append('GNSS_UTC_INVALID')
        elif values.get('_monitor_offset_parse_valid') != 'true':
            level = max(level, DiagnosticStatus.ERROR); reasons.append('GNSS_TIMING_OFFSET_MISSING_OR_MALFORMED')
        if values.get('stamp_source') != 'gnss_utc':
            level = max(level, DiagnosticStatus.WARN); reasons.append('RECEIPT_STAMP_NOT_MEASUREMENT_TIME')
        if metrics['count']:
            if (metrics['p95_sec'] > self.policy['max_gnss_receipt_difference_sec'] or
                    metrics['min_sec'] < -self.policy['future_tolerance_sec']):
                level = max(level, DiagnosticStatus.ERROR); reasons.append('PC_GNSS_CLOCK_PLUS_TRANSPORT_DIFFERENCE')
        return status_message('sensor_timing/gnss_utc', level, ', '.join(reasons) or 'GNSS_UTC_DIAGNOSTICS_OBSERVED', values)

    def publish_once(self):
        now = time.monotonic()
        with self.lock:
            host = dict(self.host)
            age = None if self.host_stamp is None else now-self.host_stamp
            ready = bool(host.get('ready')) and age is not None and age <= self.policy['host_check_stale_sec']
            host['check_receipt_age_sec'] = age
            host['ready'] = ready
            if age is None or age > self.policy['host_check_stale_sec']:
                host['status'] = 'HOST_CLOCK_CHECK_MISSING_OR_STALE'
            level = DiagnosticStatus.WARN if ready and self.sim else DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
            status = [status_message('sensor_timing/host_clock', level, host['status'], host)]
            status.extend(self._sensor_status(name, now) for name in self.windows)
            status.append(self._driver_status(now))
        self.clock_ready.publish(Bool(data=ready))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = status
        self.diagnostics.publish(array)

    def _publish_loop(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            try:
                self.publish_once()
            except Exception as error:
                self.clock_ready.publish(Bool(data=False))
                rospy.logerr_throttle(5., 'Timing monitor publication failed: %s', error)
            self.stop.wait(1./self.policy['publish_rate_hz'])


if __name__ == '__main__':
    rospy.init_node('sensor_timing_monitor')
    SensorTimingMonitor()
    rospy.spin()
