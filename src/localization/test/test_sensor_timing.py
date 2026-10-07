#!/usr/bin/python3
"""Clock readiness and timestamp semantics, without modifying clocks or requiring sensors."""
import calendar
import copy
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
from timing_core import (TimingWindow, check_host_clock, count, distribution, duration,
                         evaluate_clock, load_config, parse_chrony, parse_timesyncd)

POLICY = load_config(SCRIPTS.parent/'config/time_sync.yaml')['clock_preflight']
PROPERTIES = '''ServerName=ntp.ubuntu.com
PollIntervalUSec=34min 8s
NTPMessage={ Leap=0, DestinationTimestamp=Mon 2026-09-07 14:11:17 UTC, Ignored=no PacketCount=35 }
'''
STATUS = '''Leap: normal
Stratum: 2
Root distance: 3.462ms (max: 5s)
Offset: +5.308ms
Packet count: 35
'''
CHRONY = '''Reference ID    : AABBCCDD (ntp.example.net)
Stratum         : 3
Ref time (UTC)  : Mon Sep 07 14:11:17 2026
System time     : 0.000006523 seconds slow of NTP time
Last offset     : -0.000006747 seconds
Root delay      : 0.013639022 seconds
Root dispersion : 0.001100737 seconds
Update interval : 64.2 seconds
Leap status     : Normal
'''
EPOCH = calendar.timegm(time.strptime('2026-09-07 14:11:17', '%Y-%m-%d %H:%M:%S'))
FLAGS = {'NTP': 'yes', 'NTPSynchronized': 'yes'}


class HostClockTest(unittest.TestCase):
    def runner(self, command, timeout):
        self.assertEqual(timeout, POLICY['command_timeout_sec'])
        self.assertNotIn('set-ntp', command)
        if command[0] == 'chronyc':
            return CHRONY
        if command[1] == 'show':
            return 'NTP=yes\nNTPSynchronized=yes\n'
        if command[1] == 'show-timesync':
            return PROPERTIES
        if command[1] == 'timesync-status':
            return STATUS
        raise AssertionError(command)

    def test_timesyncd_machine_properties_and_measured_offset(self):
        sample = parse_timesyncd(PROPERTIES, STATUS)
        self.assertAlmostEqual(sample['offset_sec'], .005308)
        self.assertAlmostEqual(sample['root_distance_sec'], .003462)
        self.assertEqual(sample['sample_epoch_sec'], EPOCH)
        result = evaluate_clock(sample, FLAGS, POLICY, EPOCH+1800)
        self.assertTrue(result['ready'])
        self.assertEqual(result['sample_age_limit_sec'], 3600)

    def test_offset_distance_sync_and_freshness_independently_reject(self):
        sample = parse_timesyncd(PROPERTIES, STATUS)
        cases = [({'offset_sec': .051}, FLAGS, EPOCH+1, 'clock_offset_exceeds_limit'),
                 ({'root_distance_sec': .051}, FLAGS, EPOCH+1, 'clock_root_distance_exceeds_limit'),
                 ({}, {'NTP': 'yes', 'NTPSynchronized': 'no'}, EPOCH+1, 'kernel_clock_not_synchronized'),
                 ({}, FLAGS, EPOCH+3601, 'clock_sample_stale'),
                 ({}, FLAGS, EPOCH-5, 'clock_sample_in_future'),
                 ({'poll_interval_sec': 4096}, FLAGS, EPOCH+1, 'poll_interval_out_of_bounds')]
        for changes, flags, now, reason in cases:
            with self.subTest(reason=reason):
                result = evaluate_clock(dict(sample, **changes), flags, POLICY, now)
                self.assertFalse(result['ready']); self.assertIn(reason, result['reasons'])

    def test_chrony_external_and_local_reference(self):
        sample = parse_chrony(CHRONY)
        self.assertAlmostEqual(sample['offset_sec'], -.000006523)
        self.assertAlmostEqual(sample['root_distance_sec'], .013639022/2+.001100737)
        self.assertTrue(evaluate_clock(sample, FLAGS, POLICY, EPOCH+1)['ready'])
        local = parse_chrony(CHRONY.replace('AABBCCDD (ntp.example.net)', '7F7F0101'))
        self.assertFalse(evaluate_clock(local, FLAGS, POLICY, EPOCH+1)['ready'])
        unsynced = parse_chrony(CHRONY.replace('Normal', 'Not synchronised'))
        self.assertFalse(evaluate_clock(unsynced, FLAGS, POLICY, EPOCH+1)['ready'])

    def test_auto_backends_and_no_pps_claim(self):
        with patch('timing_core.glob.glob', return_value=[]):
            result = check_host_clock(POLICY, runner=self.runner, now=EPOCH+1, which=lambda _: None)
        self.assertTrue(result['ready']); self.assertEqual(result['backend'], 'timesyncd')
        self.assertEqual(result['pps_devices'], [])
        self.assertFalse(result['pps_discipline_verified'])
        self.assertFalse(result['hardware_sensor_sync_verified'])
        result = check_host_clock(POLICY, runner=self.runner, now=EPOCH+1, which=lambda _: '/usr/bin/chronyc')
        self.assertEqual(result['backend'], 'chrony'); self.assertTrue(result['ready'])

    def test_timeout_missing_and_nonfinite_are_not_ready(self):
        for error in [subprocess.TimeoutExpired('timedatectl', 2), FileNotFoundError('timedatectl')]:
            def fail(*args, **kwargs):
                raise error
            self.assertFalse(check_host_clock(POLICY, runner=fail)['ready'])
        with self.assertRaises(ValueError):
            parse_timesyncd(PROPERTIES.replace('DestinationTimestamp', 'absent'), STATUS)
        with self.assertRaises(ValueError):
            evaluate_clock(dict(parse_timesyncd(PROPERTIES, STATUS), offset_sec=float('nan')),
                           FLAGS, POLICY, EPOCH)

    def test_replay_exemption_does_not_check_or_claim_physical_sync(self):
        def forbidden(*args):
            raise AssertionError('host command attempted during explicit replay exemption')
        result = check_host_clock(POLICY, use_sim_time=True, runner=forbidden)
        self.assertTrue(result['ready']); self.assertEqual(result['status'], 'REPLAY_UNVERIFIED')
        self.assertFalse(result['host_clock_verified']); self.assertFalse(result['hardware_sensor_sync_verified'])

    def test_duration_parser_rejects_unknown_units(self):
        self.assertAlmostEqual(duration('-300us'), -.0003)
        self.assertEqual(duration('34min 8s'), 2048)
        for bad in ('NaNms', '5dogs', '1s junk 2ms', ''):
            with self.assertRaises(ValueError):
                duration(bad)


class TimingWindowTest(unittest.TestCase):
    def test_real_window_bound_and_percentiles(self):
        window = TimingWindow(30, 7, .3, .1)
        for i in range(100):
            window.add(100+i*.1, 100+i*.1+.02, 10+i*.1)
        snapshot = window.snapshot(19.9)
        self.assertEqual(snapshot['window_count'], 7)
        self.assertAlmostEqual(snapshot['age']['p99_sec'], .02)
        self.assertEqual(snapshot['duplicate_count'], 0)
        self.assertEqual(window.snapshot(100)['window_count'], 0)

    def test_invalid_duplicate_backwards_gap_and_clock_jump(self):
        window = TimingWindow(30, 100, .3, .1)
        self.assertFalse(window.add(float('nan'), 100, 10))
        self.assertFalse(window.add(0, 100, 10.1))
        window.add(100, 100.02, 11)
        window.add(100, 100.12, 11.1)
        window.add(99, 100.22, 11.2)
        window.add(103, 103.02, 11.3)
        result = window.snapshot(11.3)
        self.assertEqual(result['invalid_count'], 2)
        self.assertEqual(result['duplicate_count'], 1)
        self.assertEqual(result['backwards_count'], 1)
        self.assertEqual(result['gap_count'], 1)
        self.assertEqual(result['clock_jump_count'], 1)

    def test_receipt_stamp_cannot_reveal_hidden_device_latency(self):
        window = TimingWindow(30, 100, .3, .1)
        # An arbitrary earlier device measurement is absent from this interface.
        window.add(100, 100.001, 10)
        self.assertAlmostEqual(window.snapshot(10)['age']['p50_sec'], .001)
        self.assertNotIn('device_latency', window.snapshot(10))

    def test_nan_and_invalid_configuration_are_rejected(self):
        for values in [(float('nan'), 10, .3, .1), (30, 1, .3, .1), (30, 2.5, .3, .1)]:
            with self.assertRaises(ValueError):
                TimingWindow(*values)
        self.assertEqual(distribution([float('nan'), None])['count'], 0)
        for value in (0, -1, 2.5, True, float('nan')):
            with self.assertRaises(ValueError):
                count(value, 'min_samples')


class PreflightCommandTest(unittest.TestCase):
    def test_exec_preserves_driver_ros_remaps_and_never_runs_after_failure(self):
        import check_time_sync
        args = ['--config', str(SCRIPTS.parent/'config/time_sync.yaml'),
                '--exec', '/driver/node', '__name:=gps', '_port:=/dev/device']
        with patch.object(check_time_sync, 'check_host_clock', return_value={'ready': False}), \
                patch.object(check_time_sync.os, 'execvp') as execute, patch('builtins.print'):
            self.assertEqual(check_time_sync.main(args), 1)
            execute.assert_not_called()
        with patch.object(check_time_sync, 'check_host_clock', return_value={'ready': True}), \
                patch.object(check_time_sync.os, 'execvp') as execute, patch('builtins.print'):
            self.assertEqual(check_time_sync.main(args), 0)
            execute.assert_called_once_with('/driver/node', ['/driver/node', '__name:=gps', '_port:=/dev/device'])


class MonitorCallbackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import sensor_timing_monitor
        cls.module = sensor_timing_monitor

    def monitor(self):
        monitor = self.module.SensorTimingMonitor.__new__(self.module.SensorTimingMonitor)
        config = load_config(SCRIPTS.parent/'config/time_sync.yaml')
        monitor.policy = config['sensor_timing_monitor']
        monitor.lock = threading.RLock(); monitor.sim = False; monitor.gps_receipt_mode = False
        monitor.windows = {name: TimingWindow(30, 100, settings['max_gap_sec'], .1)
                           for name, settings in monitor.policy['sensors'].items()}
        monitor.driver_values = {}; monitor.driver_level = 3; monitor.driver_stamp = None
        from collections import deque
        monitor.gnss_differences = deque(maxlen=100)
        monitor.host = {'ready': True, 'status': 'HOST_CLOCK_READY', 'hardware_sensor_sync_verified': False}
        monitor.host_stamp = 100.
        monitor.clock_ready = SimpleNamespace(publish=lambda msg: self.ready.append(msg.data))
        monitor.diagnostics = SimpleNamespace(publish=lambda msg: self.diagnostics.append(msg))
        self.ready, self.diagnostics = [], []
        return monitor

    def test_heartbeat_false_when_host_result_stale_and_private_diagnostics_serializable(self):
        monitor = self.monitor()
        with patch.object(self.module.time, 'monotonic', return_value=101.), \
                patch.object(self.module.rospy.Time, 'now', return_value=self.module.rospy.Time(123)):
            monitor.publish_once()
        self.assertTrue(self.ready[-1])
        self.assertEqual(self.diagnostics[-1].header.stamp.to_sec(), 123)
        import io
        self.diagnostics[-1].serialize(io.BytesIO())
        with patch.object(self.module.time, 'monotonic', return_value=116.), \
                patch.object(self.module.rospy.Time, 'now', return_value=self.module.rospy.Time(123)):
            monitor.publish_once()
        self.assertFalse(self.ready[-1])
        self.assertEqual(self.diagnostics[-1].status[0].message, 'HOST_CLOCK_CHECK_MISSING_OR_STALE')

    def test_driver_invalid_utc_is_never_counted_as_measured_offset(self):
        monitor = self.monitor()
        status = self.module.DiagnosticStatus(name='ublox_gps: measurement timing', level=2,
                  values=[self.module.KeyValue(key='utc_valid', value='false'),
                          self.module.KeyValue(key='receipt_minus_utc_clock_offset_plus_transport_ns', value='123')])
        message = self.module.DiagnosticArray(status=[status])
        with patch.object(self.module.time, 'monotonic', return_value=101.):
            monitor._driver_callback(message)
        self.assertEqual(len(monitor.gnss_differences), 0)
        self.assertEqual(monitor._driver_status(101.).level, 2)

    def test_sensor_callback_retains_original_header_stamp(self):
        monitor = self.monitor()
        message = self.module.Imu()
        message.header.stamp = self.module.rospy.Time(122)
        with patch.object(self.module.time, 'monotonic', return_value=101.), \
                patch.object(self.module.rospy.Time, 'now', return_value=self.module.rospy.Time(123)):
            monitor._sensor_callback(message, 'imu')
        self.assertEqual(message.header.stamp.to_sec(), 122)
        self.assertEqual(monitor.windows['imu'].snapshot(101.)['age']['p50_sec'], 1.)

    def test_valid_utc_flag_without_offset_is_not_reported_ok(self):
        monitor = self.monitor()
        status = self.module.DiagnosticStatus(name='ublox_gps: measurement timing', level=0,
                  values=[self.module.KeyValue(key='utc_valid', value='true'),
                          self.module.KeyValue(key='stamp_source', value='gnss_utc')])
        with patch.object(self.module.time, 'monotonic', return_value=101.):
            monitor._driver_callback(self.module.DiagnosticArray(status=[status]))
        result = monitor._driver_status(101.)
        self.assertEqual(result.level, 2)
        self.assertIn('MISSING_OR_MALFORMED', result.message)


if __name__ == '__main__':
    unittest.main()
