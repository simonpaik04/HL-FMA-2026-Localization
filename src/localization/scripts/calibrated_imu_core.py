#!/usr/bin/python3
"""GNSS 이동 방향과 IMU를 측정 시각으로 맞추는 ROS 독립 보정 코어.

보정은 world Z 회전 offset이다. 엔코더 정지 제약은 yaw와 차량 Z 각속도에만 적용한다.
원시/정규화 토픽은 수정하지 않으며 이동 중 AHRS 드리프트 제거는 별도 문제다.
"""
import calendar
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('invalid_quaternion')
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-12 or abs(norm-1.0) > 0.001:
        raise ValueError('invalid_quaternion_norm')
    return tuple(v/norm for v in q)


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def inverse(q):
    return (-q[0], -q[1], -q[2], q[3])


def rpy(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def slerp(a, b, fraction):
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-v for v in b), -dot
    dot = min(1., dot)
    if dot > 0.9995:
        return quaternion(tuple(x+fraction*(y-x) for x, y in zip(a, b)))
    angle = math.acos(dot)
    return tuple((math.sin((1-fraction)*angle)*x+math.sin(fraction*angle)*y)/math.sin(angle)
                 for x, y in zip(a, b))


def circular_mean(values):
    return math.atan2(sum(math.sin(v) for v in values), sum(math.cos(v) for v in values))


def navpvt_stamp(message):
    """현재 ublox_gps/nav_pvt_time.h와 같은 UTC 유효성 조건; 수신 시각 대체 금지."""
    if message.valid & 7 != 7 or not message.flags2 & 32:
        raise ValueError('gnss_utc_invalid')
    if not 1970 <= message.year <= 2106 or not 0 <= message.sec <= 60:
        raise ValueError('gnss_utc_calendar_invalid')
    if not -1000000000 <= message.nano <= 1000000000:
        raise ValueError('gnss_utc_nano_invalid')
    # datetime rejects nonexistent dates instead of silently normalizing them.
    date = datetime(message.year, message.month, message.day, message.hour,
                    message.min, min(message.sec, 59))
    seconds = calendar.timegm(date.timetuple()) + (message.sec == 60)
    nanoseconds = seconds*1000000000 + message.nano
    if not 0 < nanoseconds <= 4294967295999999999:
        raise ValueError('gnss_utc_out_of_ros_range')
    return nanoseconds/1e9


@dataclass(frozen=True)
class GnssSample:
    stamp: float
    course: float  # ENU, east=0, CCW positive
    speed: float
    heading_accuracy: float  # rad
    horizontal_accuracy: float  # m
    speed_accuracy: float
    latitude: float
    longitude: float
    satellites: int
    fix_ok: bool
    velocity_course: float
    time_accuracy: float = 0.0

    @classmethod
    def from_navpvt(cls, message):
        return cls(navpvt_stamp(message), wrap(math.pi/2-math.radians(message.heading*1e-5)),
                   message.gSpeed*1e-3, math.radians(message.headAcc*1e-5),
                   message.hAcc*1e-3, message.sAcc*1e-3,
                   message.lat*1e-7, message.lon*1e-7, message.numSV,
                   bool(message.flags & 1) and message.fixType == 3,
                   math.atan2(message.velN, message.velE), message.tAcc*1e-9)


class EncoderYawHold:
    """Header 없는 feedback의 수신 이력으로 정지 시점 yaw를 유지한다."""
    def __init__(self, policy):
        self.enabled = policy['enabled']
        self.max_age = policy['max_feedback_age_sec']
        count = policy['history_max_samples']
        if not isinstance(self.enabled, bool):
            raise ValueError('encoder_yaw_hold/enabled must be boolean')
        if isinstance(self.max_age, bool) or not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('invalid encoder_yaw_hold/max_feedback_age_sec')
        if isinstance(count, bool) or not math.isfinite(count) or count < 2 or int(count) != count:
            raise ValueError('invalid encoder_yaw_hold/history_max_samples')
        self.history_size = int(count)
        self.reset()

    def reset(self):
        self.feedback = deque(maxlen=self.history_size)
        self.last_alive = None
        self.held = False
        self.offset = 0.
        self.last_yaw = None
        self.reason = 'ENCODER_NOT_RECEIVED' if self.enabled else 'DISABLED'
        self.encoder_age = None
        self.held_samples = 0

    def observe_feedback(self, stamp, encoder, speed, alive):
        if not math.isfinite(stamp) or stamp <= 0 or (self.feedback and stamp <= self.feedback[-1][0]):
            return False
        valid = math.isfinite(speed) and alive != self.last_alive
        self.last_alive = alive
        # 모순된 speed!=0/encoder=0 또는 중복 alive를 정지 증거로 사용하지 않는다.
        state = ('ZERO' if encoder == 0 and speed == 0 else 'MOVING') if valid else 'INVALID'
        if encoder == 0 and speed != 0:
            state = 'INVALID'
        self.feedback.append((stamp, state))
        return valid

    def apply(self, stamp, q, gyro, now, mount):
        if not self.enabled:
            return q, gyro
        evidence = next((row for row in reversed(self.feedback) if row[0] <= stamp), None)
        self.encoder_age = None if evidence is None else stamp-evidence[0]
        state = 'ENCODER_NOT_RECEIVED'
        if evidence is not None:
            state = evidence[1] if max(stamp, now)-evidence[0] <= self.max_age else 'ENCODER_STALE'
        body_yaw = rpy(multiply(q, inverse(mount)))[2]
        stopped = state == 'ZERO'
        if self.last_yaw is not None and (stopped or (self.held and state == 'MOVING')):
            # 정지 중 drift를 offset에 흡수한다. 첫 재출발 표본도 이전 yaw에서 연결한다.
            self.offset = wrap(self.last_yaw-body_yaw)
        rotation = (0., 0., math.sin(self.offset/2), math.cos(self.offset/2))
        result = multiply(rotation, q) if self.offset else q
        self.last_yaw = rpy(multiply(result, inverse(mount)))[2]
        self.held = stopped
        self.reason = 'ENCODER_ZERO' if stopped else state
        if stopped:
            self.held_samples += 1
            # 차량 Z축을 센서 좌표로 표현해 그 성분만 제거한다. 장착 TF는 변경하지 않는다.
            axis = multiply(multiply(inverse(mount), (0., 0., 1., 0.)), mount)[:3]
            rate = sum(a*v for a, v in zip(axis, gyro))
            gyro = tuple(v-rate*a for a, v in zip(axis, gyro))
        return result, gyro

    def status(self):
        return dict(encoder_yaw_held=self.held, encoder_yaw_hold_reason=self.reason,
                    encoder_yaw_hold_age_sec=self.encoder_age,
                    stationary_yaw_offset_deg=math.degrees(self.offset),
                    stationary_held_samples=self.held_samples)


class HeadingCalibration:
    """선택한 시작 방향으로 초기화한 뒤 GNSS 정합을 허용한다. 원본 기록은 보존한다."""
    def __init__(self, policy, initial_heading=None, encoder_yaw_hold=None):
        self.p = dict(policy)
        self.yaw_hold = EncoderYawHold(encoder_yaw_hold) if encoder_yaw_hold is not None else None
        self.initial_heading = dict(initial_heading) if initial_heading is not None else None
        if self.initial_heading is not None:
            for key in ('yaw_rad', 'standard_deviation_deg'):
                value = self.initial_heading[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError('invalid initial_heading/'+key)
            if not 0 < self.initial_heading['standard_deviation_deg'] <= 180.:
                raise ValueError('invalid initial heading uncertainty')
            if not isinstance(self.initial_heading['source'], str) or not self.initial_heading['source']:
                raise ValueError('initial heading source required')
        for name, value in self.p.items():
            if name in ('enabled', 'one_shot'):
                if not isinstance(value, bool):
                    raise ValueError(name+' must be boolean')
            elif name == 'direction_mode':
                if value not in ('forward_start', 'signed_encoder'):
                    raise ValueError('unknown direction_mode')
            elif isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')
        for name in ('history_max_samples', 'pending_max_samples', 'min_samples', 'min_satellites'):
            if int(self.p[name]) != self.p[name]:
                raise ValueError(name+' must be integer')
        if self.p['min_samples'] < 2 or self.p['history_max_samples'] < 2:
            raise ValueError('sample bounds too small')
        if self.p['history_sec'] <= self.p['max_gnss_age_sec']:
            raise ValueError('history must cover delayed GNSS')
        if self.p['min_window_sec'] > self.p['max_window_sec']:
            raise ValueError('minimum window exceeds maximum')
        if not 0. < self.p['max_repeat_heading_difference_deg'] < 90.:
            raise ValueError('max_repeat_heading_difference_deg must be between 0 and 90')
        self.reset()

    def reset(self):
        if self.yaw_hold is not None:
            self.yaw_hold.reset()
        self.imus = deque(maxlen=int(self.p['history_max_samples']))
        self.twists = deque(maxlen=int(self.p['history_max_samples']))
        self.window = deque(maxlen=int(self.p['history_max_samples']))
        self.calibrated = False
        self.initialized = False
        self.initialization_stamp = None
        self.offset = 0.0
        self.applied_offset = 0.0
        self.last_output_stamp = None
        self.variance = 0.0
        self.calibration_stamp = None
        self.last_gnss_stamp = None
        self.last_now = None
        self.reason = 'WAITING_FOR_GNSS'
        self.correction_count = 0

    def initialize_heading(self, mount):
        """첫 유효 IMU의 차량 yaw를 시작 방향에 맞춘다. GNSS one-shot을 소비하지 않는다."""
        if self.initial_heading is None or self.initialized or self.calibrated or not self.imus:
            return False
        stamp, q, _ = self.imus[-1]
        base_roll, base_pitch, base_yaw = rpy(multiply(q, inverse(quaternion(mount))))
        if max(abs(base_roll), abs(base_pitch), abs(rpy(q)[1])) > math.radians(self.p['max_tilt_deg']):
            self.reason = 'INITIAL_HEADING_TILT_TOO_LARGE'
            return False
        self.offset = wrap(self.initial_heading['yaw_rad']-base_yaw)
        # 발행 전 초기화이므로 여기에는 주행 중 GNSS 보정의 slew를 적용하지 않는다.
        self.applied_offset = self.offset
        self.initialized = True
        self.initialization_stamp = stamp
        self.variance = math.radians(self.initial_heading['standard_deviation_deg'])**2
        self.reason = 'RDDF_INITIALIZED'
        return True

    def select_initial_heading(self, yaw_rad, source, now, mount, standard_deviation_deg=10.):
        """RDDF 선택 후 최신 유효 IMU와 한 번 정합한다. 이동 중 재보정 API가 아니다."""
        if self.initialized or self.calibrated:
            return False
        if (not math.isfinite(yaw_rad) or not math.isfinite(standard_deviation_deg) or
                not 0 < standard_deviation_deg <= 180 or not source or not self.imus or
                not 0 <= now-self.imus[-1][0] <= self.p['max_imu_age_sec']):
            return False
        self.initial_heading = dict(yaw_rad=wrap(yaw_rad), standard_deviation_deg=standard_deviation_deg, source=source)
        return self.initialize_heading(mount)

    def observe_time(self, now):
        """ROS clock rollback starts a new bag/session; reordered sensors do not reset it."""
        if not math.isfinite(now) or now <= 0:
            return False
        if self.last_now is not None and now < self.last_now-1e-6:
            self.reset()
            self.reason = 'CLOCK_ROLLBACK_RESET'
        self.last_now = now
        return True

    def _prune(self, history, stamp):
        while history and history[0][0] < stamp-self.p['history_sec']:
            history.popleft()

    def observe_feedback(self, stamp, encoder, speed, alive):
        if not self.observe_time(stamp) or self.yaw_hold is None:
            return False
        return self.yaw_hold.observe_feedback(stamp, encoder, speed, alive)

    def observe_imu(self, stamp, q, gyro, now, mount=(0., 0., 0., 1.)):
        if not self.observe_time(now):
            return False
        if (not math.isfinite(stamp) or stamp <= 0 or
                not -self.p['max_future_sec'] <= now-stamp <= self.p['max_imu_age_sec'] or
                (self.imus and stamp <= self.imus[-1][0]) or
                len(gyro) != 3 or not all(math.isfinite(v) for v in gyro)):
            return False
        try:
            q = quaternion(q)
            mount = quaternion(mount)
        except ValueError:
            return False
        if self.yaw_hold is not None:
            q, gyro = self.yaw_hold.apply(stamp, q, tuple(gyro), now, mount)
        self.imus.append((stamp, q, tuple(gyro)))
        self._prune(self.imus, stamp)
        return True

    def observe_twist(self, stamp, speed, now):
        if not self.observe_time(now):
            return False
        if (not all(math.isfinite(v) for v in (stamp, speed)) or stamp <= 0 or
                not -self.p['max_future_sec'] <= now-stamp <= self.p['max_encoder_age_sec'] or
                (self.twists and stamp <= self.twists[-1][0])):
            return False
        self.twists.append((stamp, speed))
        self._prune(self.twists, stamp)
        return True

    def _imu_at(self, stamp):
        for index, row in enumerate(self.imus):
            if abs(row[0]-stamp) < 1e-7:
                return row[1:]
            if row[0] > stamp and index:
                previous = self.imus[index-1]
                gap = row[0]-previous[0]
                if gap > self.p['max_imu_gap_sec']:
                    return None
                fraction = (stamp-previous[0])/gap
                return (slerp(previous[1], row[1], fraction),
                        tuple(x+fraction*(y-x) for x, y in zip(previous[2], row[2])))
        return None

    def _speed_at(self, stamp):
        # Causal last observation, with a strict age bound; no latest-at-receipt shortcut.
        for timestamp, speed in reversed(self.twists):
            if timestamp <= stamp:
                return speed if stamp-timestamp <= self.p['max_encoder_age_sec'] else None
        return None

    def reject(self, reason):
        self.window.clear()
        self.reason = reason
        return reason

    def observe_gnss(self, sample, now, clock_ready, mount=(0., 0., 0., 1.)):
        if not self.observe_time(now):
            return self.reject('CLOCK_INVALID')
        if self.initial_heading is not None and not self.initialized:
            return self.reject('WAITING_FOR_INITIAL_HEADING')
        if self.calibrated and self.p['one_shot']:
            self.reason = 'HOLDING_CALIBRATION'
            return self.reason
        if not self.p['enabled']:
            return self.reject('CALIBRATION_DISABLED')
        if not clock_ready:
            return self.reject('CLOCK_NOT_READY')
        values = (sample.stamp, sample.course, sample.speed, sample.heading_accuracy,
                  sample.horizontal_accuracy, sample.speed_accuracy, sample.latitude,
                  sample.longitude, sample.velocity_course, sample.time_accuracy)
        if not all(math.isfinite(v) for v in values) or sample.stamp <= 0:
            return self.reject('GNSS_NONFINITE')
        if not -self.p['max_future_sec'] <= now-sample.stamp <= self.p['max_gnss_age_sec']:
            return self.reject('GNSS_STAMP_OUT_OF_RANGE')
        if not 0 <= sample.time_accuracy <= self.p['max_gnss_time_accuracy_sec']:
            return self.reject('GNSS_TIME_ACCURACY_REJECTED')
        if self.last_gnss_stamp is not None and sample.stamp <= self.last_gnss_stamp:
            return self.reject('GNSS_DUPLICATE_OR_REORDERED')
        if not sample.fix_ok or sample.satellites < self.p['min_satellites']:
            return self.reject('GNSS_FIX_INVALID')
        if (not -90 <= sample.latitude <= 90 or not -180 <= sample.longitude <= 180 or
                not 0 < sample.horizontal_accuracy <= self.p['max_horizontal_accuracy_m'] or
                not 0 < sample.heading_accuracy <= math.radians(self.p['max_heading_accuracy_deg']) or
                not 0 < sample.speed_accuracy <= self.p['max_speed_accuracy_mps']):
            return self.reject('GNSS_ACCURACY_REJECTED')
        if sample.speed < self.p['min_speed_mps']:
            return self.reject('GNSS_TOO_SLOW')
        if abs(wrap(sample.course-sample.velocity_course)) > math.radians(self.p['max_velocity_heading_difference_deg']):
            return self.reject('GNSS_VELOCITY_HEADING_MISMATCH')
        aligned = self._imu_at(sample.stamp)
        if aligned is None:
            return 'WAITING_FOR_IMU_ALIGNMENT'
        speed = self._speed_at(sample.stamp)
        if speed is None:
            return 'WAITING_FOR_ENCODER_ALIGNMENT'
        self.last_gnss_stamp = sample.stamp
        if abs(speed) < self.p['min_encoder_speed_mps']:
            return self.reject('ENCODER_TOO_SLOW')
        if self.p['direction_mode'] == 'forward_start' and speed < 0:
            return self.reject('REVERSE_DURING_FORWARD_START')
        if abs(abs(speed)-sample.speed) > max(self.p['max_speed_difference_mps'],
                                            sample.speed*self.p['max_speed_difference_ratio']):
            return self.reject('ENCODER_GNSS_SPEED_MISMATCH')
        q, gyro = aligned
        roll, pitch, _ = rpy(q)
        base = multiply(q, inverse(quaternion(mount)))
        base_roll, base_pitch, base_yaw = rpy(base)
        if max(abs(base_roll), abs(base_pitch), abs(pitch)) > math.radians(self.p['max_tilt_deg']):
            return self.reject('TILT_TOO_LARGE')
        yaw_rate = (math.sin(roll)*gyro[1]+math.cos(roll)*gyro[2])/math.cos(pitch)
        if abs(yaw_rate) > math.radians(self.p['max_yaw_rate_degps']):
            return self.reject('TURNING')
        # Reverse motion differs from body heading by pi, only with verified signed speed.
        direction = 1 if speed > 0 else -1
        body_course = wrap(sample.course+(math.pi if direction < 0 else 0.))
        candidate = wrap(body_course-base_yaw)
        if self.calibrated and not self.p['one_shot']:
            # Reuse the established body heading as a consistency bound; an
            # unsigned encoder or constant Gear must not turn reverse COG into
            # a new forward heading. This does not certify motion direction.
            if abs(wrap(self.offset-self.applied_offset)) > 1e-6:
                return self.reject('WAITING_FOR_PREVIOUS_ALIGNMENT')
            if abs(wrap(candidate-self.offset)) > math.radians(self.p['max_repeat_heading_difference_deg']):
                return self.reject('REPEAT_HEADING_DIFFERENCE_TOO_LARGE')
        if self.window and (sample.stamp-self.window[-1][0].stamp > self.p['max_gnss_gap_sec'] or
                            direction != self.window[-1][2]):
            self.window.clear()
        self.window.append((sample, candidate, direction))
        while self.window and sample.stamp-self.window[0][0].stamp > self.p['max_window_sec']:
            self.window.popleft()
        courses = [row[0].course for row in self.window]
        offsets = [row[1] for row in self.window]
        mean_course, mean_offset = circular_mean(courses), circular_mean(offsets)
        course_spread = max(wrap(v-mean_course) for v in courses)-min(wrap(v-mean_course) for v in courses)
        offset_spread = max(wrap(v-mean_offset) for v in offsets)-min(wrap(v-mean_offset) for v in offsets)
        if course_spread > math.radians(self.p['max_course_spread_deg']):
            return self.reject('COURSE_NOT_STRAIGHT')
        if offset_spread > math.radians(self.p['max_offset_spread_deg']):
            return self.reject('OFFSET_NOT_STABLE')
        first = self.window[0][0]
        if len(self.window) < self.p['min_samples'] or sample.stamp-first.stamp < self.p['min_window_sec']:
            self.reason = 'COLLECTING_STRAIGHT_MOTION'
            return self.reason
        north = math.radians(sample.latitude-first.latitude)*6378137.
        east = wrap(math.radians(sample.longitude-first.longitude))*6378137.*math.cos(math.radians((sample.latitude+first.latitude)/2))
        if math.hypot(east, north) < self.p['min_displacement_m']:
            self.reason = 'WAITING_FOR_DISPLACEMENT'
            return self.reason
        if abs(wrap(math.atan2(north, east)-mean_course)) > math.radians(self.p['max_displacement_course_difference_deg']):
            return self.reject('GPS_POSITION_COURSE_MISMATCH')
        self.offset = mean_offset
        # The floor and worst headAcc are retained: adjacent GNSS fixes are correlated.
        self.variance = max(math.radians(self.p['heading_stddev_floor_deg']),
                            max(row[0].heading_accuracy for row in self.window))**2
        self.calibrated = True
        self.calibration_stamp = sample.stamp
        self.correction_count += 1
        self.window.clear()
        self.reason = 'CALIBRATED'
        return self.reason

    def output_orientation(self, q):
        if self.initial_heading is not None and not self.initialized:
            raise ValueError('WAITING_FOR_INITIAL_HEADING')
        if not self.calibrated and not self.initialized:
            return tuple(q)
        rotation = (0., 0., math.sin(self.offset/2), math.cos(self.offset/2))
        return multiply(rotation, quaternion(q))

    def step_output_orientation(self, q, stamp):
        """큰 최초 보정각이 EKF yaw 이상치 gate에 걸리지 않도록 적용 속도를 제한한다."""
        if self.initial_heading is not None and not self.initialized:
            raise ValueError('WAITING_FOR_INITIAL_HEADING')
        elapsed = 0. if self.last_output_stamp is None else max(0., stamp-self.last_output_stamp)
        self.last_output_stamp = stamp
        if not self.calibrated and not self.initialized:
            return tuple(q)
        # IMU 단절 시간 동안 보정을 몰아서 적용하지 않는다.
        step = math.radians(self.p['max_correction_rate_degps'])*min(elapsed, self.p['max_imu_gap_sec'])
        if self.yaw_hold is not None and self.yaw_hold.held:
            step = 0.  # 정지 중에는 지연 GNSS 정합에 따른 방향 변경도 재출발까지 미룬다.
        error = wrap(self.offset-self.applied_offset)
        self.applied_offset = wrap(self.applied_offset+max(-step, min(step, error)))
        rotation = (0., 0., math.sin(self.applied_offset/2), math.cos(self.applied_offset/2))
        return multiply(rotation, quaternion(q))

    def added_yaw_variance(self, stamp):
        if not self.calibrated and not self.initialized:
            return 0.0
        reference_stamp = self.calibration_stamp if self.calibrated else self.initialization_stamp
        elapsed = max(0., stamp-reference_stamp)
        uncertainty = math.radians(self.p['held_heading_uncertainty_deg_per_sec'])*elapsed
        remaining = wrap(self.offset-self.applied_offset)
        return min(math.pi**2, self.variance+uncertainty**2+remaining**2)

    def status(self):
        applied = abs(wrap(self.offset-self.applied_offset)) < 1e-6
        initial_state = ('RDDF_INITIALIZED' if self.initialized else 'WAITING_FOR_INITIAL_HEADING') if self.initial_heading is not None else 'UNCALIBRATED'
        return {'state': ('CALIBRATED' if applied else 'ALIGNING') if self.calibrated else initial_state,
                'reason': self.reason, 'yaw_offset_deg': math.degrees(self.offset),
                'applied_yaw_offset_deg': math.degrees(self.applied_offset),
                'initial_heading_source': None if self.initial_heading is None else self.initial_heading['source'],
                'initial_yaw_deg': None if self.initial_heading is None else math.degrees(self.initial_heading['yaw_rad']),
                'initialization_stamp': self.initialization_stamp,
                'calibration_stamp': self.calibration_stamp,
                'correction_count': self.correction_count, 'candidate_samples': len(self.window),
                'direction_mode': self.p['direction_mode'], 'one_shot': self.p['one_shot']}
