#!/usr/bin/python3
"""CalibratedIMU 공통 출구. 시작 방향/GNSS 정합과 엔코더 정지 yaw 제약을 적용한다."""
from collections import deque
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from calibrated_imu_core import GnssSample, HeadingCalibration, quaternion

import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from ublox_msgs.msg import NavPVT
from mando_localization.srv import SetInitialHeading, SetInitialHeadingResponse


class CalibratedIMU:
    def __init__(self):
        self.core = HeadingCalibration(rospy.get_param('~imu_heading_calibration'),
                                       rospy.get_param('~initial_heading', None),
                                       rospy.get_param('~encoder_yaw_hold'))
        self.wait_for_rddf = rospy.get_param("~initialization/required", False)
        if self.wait_for_rddf:
            self.core.initial_heading = None
        self.heading_transaction = None
        self.p = self.core.p
        self.topics = rospy.get_param('~topics')
        self.frames = rospy.get_param('~frames')
        for key in ('imu_normalized', 'imu_calibrated', 'gps_navpvt', 'encoder_twist', 'encoder_state'):
            if not isinstance(self.topics[key], str) or not self.topics[key].startswith('/'):
                raise ValueError(key+' must be an absolute ROS topic')
        if len({self.topics[k] for k in ('imu_data', 'imu_normalized', 'imu_calibrated')}) != 3:
            raise ValueError('raw, normalized and calibrated IMU topics must differ')
        self.lock = threading.RLock()
        self.pending = deque()
        self.mount = None
        self.clock_value = False
        self.clock_receipt = None
        self.clock_mono = None
        self.last_now = None
        self.last_imu_mono = None
        self.published = 0
        self.rejected_imu = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(self.topics['imu_calibrated'], Imu, queue_size=50)
        self.diagnostics = rospy.Publisher(rospy.get_param('~internal_topics/imu_calibration_status'),
                                           DiagnosticArray, queue_size=2, latch=True)
        self.subscribers = [
            rospy.Subscriber(self.topics['imu_normalized'], Imu, self.imu_callback, queue_size=200),
            rospy.Subscriber(self.topics['gps_navpvt'], NavPVT, self.gnss_callback, queue_size=30),
            rospy.Subscriber(self.topics['encoder_twist'], TwistWithCovarianceStamped,
                             self.twist_callback, queue_size=100),
            rospy.Subscriber(self.topics['encoder_state'], SerialFeedBack,
                             self.feedback_callback, queue_size=100),
            rospy.Subscriber(rospy.get_param('~internal_topics/clock_ready'), Bool,
                             self.clock_callback, queue_size=5),
        ]
        self.heading_service = rospy.Service("~set_initial_heading", SetInitialHeading, self.set_initial_heading)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback, reset=True)
        rospy.loginfo('CalibratedIMU: %s -> %s; direction_mode=%s, one_shot=%s',
                      self.topics['imu_normalized'], self.topics['imu_calibrated'],
                      self.p['direction_mode'], self.p['one_shot'])
        if self.core.initial_heading is not None:
            rospy.loginfo('Initial body yaw: %.6f deg from %s; vehicle must face this start direction.',
                          math.degrees(self.core.initial_heading['yaw_rad']),
                          self.core.initial_heading['source'])
        if self.p['direction_mode'] == 'forward_start':
            rospy.logwarn('Initial yaw alignment assumes forward straight motion until calibrated; '
                          'Gear is not a verified direction signal.')

    def set_initial_heading(self, request):
        with self.lock:
            now = self._time()
            signature = (request.transaction_id, request.yaw_rad, request.source, request.standard_deviation_deg)
            if not self.wait_for_rddf or request.transaction_id == 0:
                return SetInitialHeadingResponse(False, 'DYNAMIC_INITIALIZATION_DISABLED', rospy.Time())
            if self.heading_transaction is not None:
                okay = signature == self.heading_transaction and self.core.initialized
                return SetInitialHeadingResponse(okay, 'ALREADY_INITIALIZED',
                    rospy.Time.from_sec(self.core.initialization_stamp or 0.))
            if (self.last_imu_mono is None or time.monotonic()-self.last_imu_mono > self.p["max_imu_age_sec"]):
                return SetInitialHeadingResponse(False, "WAITING_FOR_FRESH_IMU", rospy.Time())
            if not self._mount() or not self.core.select_initial_heading(
                    request.yaw_rad, request.source, now, self.mount, request.standard_deviation_deg):
                return SetInitialHeadingResponse(False, 'WAITING_FOR_FRESH_IMU', rospy.Time())
            self.heading_transaction = signature
            return SetInitialHeadingResponse(True, 'RDDF_INITIALIZED',
                rospy.Time.from_sec(self.core.initialization_stamp))

    def _time(self):
        now = rospy.Time.now().to_sec()
        if self.last_now is not None and now < self.last_now-1e-6:
            self.pending.clear()
            self.heading_transaction = None
            if self.wait_for_rddf:
                self.core.initial_heading = None
            self.clock_value = False
            self.clock_receipt = None
            self.clock_mono = None
        self.last_now = now
        self.core.observe_time(now)
        return now

    def _clock_ready(self, now):
        return (self.clock_value and self.clock_receipt is not None and
                0 <= now-self.clock_receipt <= self.p['clock_ready_timeout_sec'] and
                time.monotonic()-self.clock_mono <= self.p['clock_ready_timeout_sec'])

    def _mount(self):
        if self.mount is None:
            try:
                transform = self.tf_buffer.lookup_transform(self.frames['base_link'], self.frames['imu'], rospy.Time(0))
                q = transform.transform.rotation
                self.mount = quaternion((q.x, q.y, q.z, q.w))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, ValueError):
                self.core.reject('WAITING_FOR_IMU_MOUNT_TF')
                return False
        return True

    def clock_callback(self, message):
        with self.lock:
            self.clock_receipt = self._time()
            self.clock_mono = time.monotonic()
            self.clock_value = bool(message.data)
            if not self.clock_value:
                self.pending.clear()
                self.core.reject('CLOCK_NOT_READY')

    def twist_callback(self, message):
        with self.lock:
            now = self._time()
            if message.header.frame_id != self.frames['base_link']:
                return
            self.core.observe_twist(message.header.stamp.to_sec(), message.twist.twist.linear.x, now)
            self._pending(now)

    def feedback_callback(self, message):
        with self.lock:
            self.core.observe_feedback(self._time(), message.encoder, message.speed, message.alive)

    @staticmethod
    def valid_payload(message):
        vectors = (message.angular_velocity, message.linear_acceleration)
        if not all(math.isfinite(v) for vector in vectors for v in (vector.x, vector.y, vector.z)):
            return False
        for covariance in (message.orientation_covariance, message.angular_velocity_covariance,
                           message.linear_acceleration_covariance):
            if not all(math.isfinite(v) for v in covariance) or any(covariance[i] <= 0 for i in (0, 4, 8)):
                return False
        return True

    def imu_callback(self, message):
        with self.lock:
            now = self._time()
            q, gyro = message.orientation, message.angular_velocity
            if self.core.yaw_hold.enabled and not self._mount():
                return
            if (message.header.frame_id != self.frames['imu'] or not self.valid_payload(message) or
                    not self.core.observe_imu(message.header.stamp.to_sec(), (q.x, q.y, q.z, q.w),
                                              (gyro.x, gyro.y, gyro.z), now,
                                              self.mount or (0., 0., 0., 1.))):
                self.rejected_imu += 1
                return
            self.last_imu_mono = time.monotonic()
            if self.wait_for_rddf and self.heading_transaction is None:
                self.core.reason = "WAITING_FOR_RDDF_POSITION"
                return
            if self.core.initial_heading is not None and not self.core.initialized:
                if not self._mount() or not self.core.initialize_heading(self.mount):
                    return
            self._pending(now)
            output = copy.deepcopy(message)
            _, guarded_q, guarded_gyro = self.core.imus[-1]
            result = self.core.step_output_orientation(guarded_q, message.header.stamp.to_sec())
            output.orientation.x, output.orientation.y, output.orientation.z, output.orientation.w = result
            output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = guarded_gyro
            if self.core.calibrated or self.core.initialized:
                covariance = list(output.orientation_covariance)
                covariance[8] += self.core.added_yaw_variance(message.header.stamp.to_sec())
                output.orientation_covariance = covariance
            self.publisher.publish(output)
            self.published += 1

    def gnss_callback(self, message):
        with self.lock:
            now = self._time()
            if self.wait_for_rddf and self.heading_transaction is None:
                return
            if self.core.calibrated and self.p['one_shot']:
                return
            try:
                sample = GnssSample.from_navpvt(message)
            except (ValueError, TypeError, OverflowError) as error:
                self.pending.clear()
                self.core.reject(str(error))
                return
            if len(self.pending) >= self.p['pending_max_samples']:
                self.pending.popleft()
                self.core.reject('GNSS_PENDING_OVERFLOW')
            self.pending.append((sample, time.monotonic()))
            self._pending(now)

    def _pending(self, now):
        if not self.pending:
            return
        if not self._clock_ready(now):
            self.pending.clear()
            self.core.reject('CLOCK_NOT_READY')
            return
        if not self._mount():
            self.pending.clear()
            return
        while self.pending:
            sample, received = self.pending[0]
            reason = self.core.observe_gnss(sample, now, True, self.mount)
            if reason in ('WAITING_FOR_IMU_ALIGNMENT', 'WAITING_FOR_ENCODER_ALIGNMENT'):
                self.core.reason = reason
                if time.monotonic()-received <= self.p['pending_wait_sec']:
                    break
                self.core.reject(reason+'_TIMEOUT')
            self.pending.popleft()
            if reason == 'CALIBRATED':
                rospy.loginfo('GNSS yaw alignment accepted: offset=%.3f deg, gradual application at %.2f deg/s',
                              math.degrees(self.core.offset), self.p['max_correction_rate_degps'])

    def timer_callback(self, _event):
        with self.lock:
            now = self._time()
            self._pending(now)
            values = self.core.status()
            values.update(self.core.yaw_hold.status())
            values.update(published_samples=self.published, rejected_imu_samples=self.rejected_imu,
                          clock_ready=self._clock_ready(now),
                          heading_accuracy_verified=False, hardware_sensor_sync_verified=False,
                          original_imu_preserved=True)
            age = None if self.last_imu_mono is None else time.monotonic()-self.last_imu_mono
            values['imu_receipt_age_sec'] = age
            status = DiagnosticStatus(name='CalibratedIMU', hardware_id='imu_gnss_heading_alignment')
            status.level = DiagnosticStatus.OK if values['state'] == 'CALIBRATED' else DiagnosticStatus.WARN
            status.message = values['state']+': '+values['reason']
            if age is None or age > self.p['max_imu_gap_sec']*3:
                status.level = DiagnosticStatus.ERROR
                status.message = 'IMU_NOT_RECEIVED' if age is None else 'IMU_STALE'
            status.values = [KeyValue(key=key, value=json.dumps(value, allow_nan=False))
                             for key, value in sorted(values.items())]
            array = DiagnosticArray()
            array.header.stamp = rospy.Time.now()
            array.status = [status]
            self.diagnostics.publish(array)


if __name__ == '__main__':
    rospy.init_node('calibrated_imu')
    try:
        node = CalibratedIMU()
        rospy.spin()
    except (KeyError, ValueError, rospy.ROSException) as error:
        rospy.logfatal('CalibratedIMU configuration failed: %s', error)
        sys.exit(1)
