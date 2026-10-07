#!/usr/bin/python3
"""정지 드리프트, 재출발, 엔코더 단절과 장착 좌표계 회귀 시험."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_calibrated_imu import orientation, policy
from calibrated_imu_core import HeadingCalibration, inverse, multiply, quaternion, rpy, wrap


class EncoderYawHoldTest(unittest.TestCase):
    def setUp(self):
        self.hold_policy = dict(enabled=True, max_feedback_age_sec=.3, history_max_samples=500)
        self.core = HeadingCalibration(policy(), encoder_yaw_hold=self.hold_policy)
        self.alive = 0

    def feedback(self, stamp, encoder=0, speed=0.):
        self.alive = (self.alive+1) % 256
        self.core.observe_feedback(stamp, encoder, speed, self.alive)

    def imu(self, stamp, angle, mount=(0., 0., 0., 1.), roll=0., pitch=0., gyro=(.1, .2, .3), now=None):
        self.assertTrue(self.core.observe_imu(stamp, orientation(roll, pitch, angle), gyro,
                                             stamp if now is None else now, mount))
        _, q, corrected_gyro = self.core.imus[-1]
        return self.core.step_output_orientation(q, stamp), corrected_gyro

    def test_zero_holds_yaw_and_removes_drift_before_restart(self):
        self.feedback(100., encoder=5, speed=1.)
        self.imu(100.01, 170.)
        for index in range(1, 101):
            stamp = 100.+index*.1
            self.feedback(stamp)
            q, gyro = self.imu(stamp+.01, 170.+index*.8, roll=5., pitch=8.)
            self.assertAlmostEqual(wrap(rpy(q)[2]-math.radians(170.)), 0.)
            self.assertAlmostEqual(rpy(q)[0], math.radians(5.))
            self.assertAlmostEqual(rpy(q)[1], math.radians(8.))
            self.assertEqual(gyro, (.1, .2, 0.))
        self.feedback(110.1, encoder=-1, speed=-.02)
        q, gyro = self.imu(110.11, 260.)
        self.assertAlmostEqual(wrap(rpy(q)[2]-math.radians(170.)), 0.)
        self.assertEqual(gyro, (.1, .2, .3))
        q, _ = self.imu(110.12, 265.)
        self.assertAlmostEqual(wrap(rpy(q)[2]-math.radians(175.)), 0.)
        self.assertFalse(self.core.yaw_hold.held)

    def test_missing_stale_and_future_feedback_do_not_lock(self):
        self.imu(100., 10.)
        q, gyro = self.imu(100.1, 20.)
        self.assertAlmostEqual(rpy(q)[2], math.radians(20.))
        self.assertEqual(gyro[2], .3)
        self.feedback(100.2)
        q, _ = self.imu(100.19, 30., now=100.2)
        self.assertAlmostEqual(rpy(q)[2], math.radians(30.))
        self.imu(100.21, 40.)
        q, gyro = self.imu(100.51, 60.)
        self.assertFalse(self.core.yaw_hold.held)
        self.assertEqual(gyro[2], .3)
        self.assertAlmostEqual(rpy(q)[2], math.radians(50.))

    def test_duplicate_alive_and_conflicting_speed_invalidate_zero(self):
        self.feedback(100.)
        self.imu(100.01, 10.)
        self.core.observe_feedback(100.1, 0, 0., self.alive)
        self.imu(100.11, 20.)
        self.assertFalse(self.core.yaw_hold.held)
        self.feedback(100.2, encoder=0, speed=1.)
        self.imu(100.21, 30.)
        self.assertFalse(self.core.yaw_hold.held)
        self.feedback(100.3, speed=math.nan)
        self.imu(100.31, 40.)
        self.assertFalse(self.core.yaw_hold.held)

    def test_feedback_history_is_causal_even_if_newer_motion_arrives_first(self):
        self.feedback(100.)
        self.imu(100.01, 10.)
        self.feedback(100.1, encoder=1, speed=.02)
        q, _ = self.imu(100.09, 20., now=100.1)
        self.assertTrue(self.core.yaw_hold.held)
        self.assertAlmostEqual(rpy(q)[2], math.radians(10.))

    def test_tilted_mount_holds_vehicle_yaw_and_vehicle_z_rate(self):
        mount = orientation(roll=20., pitch=10., yaw=30.)
        self.feedback(100.)
        q, gyro = self.imu(100.01, 20., mount=mount, roll=10., pitch=5.)
        initial_yaw = rpy(multiply(q, inverse(mount)))[2]
        q, gyro = self.imu(100.02, 40., mount=mount, roll=15., pitch=8.)
        self.assertAlmostEqual(wrap(rpy(multiply(q, inverse(mount)))[2]-initial_yaw), 0.)
        rotated = multiply(multiply(mount, (*gyro, 0.)), inverse(mount))
        original = multiply(multiply(mount, (.1, .2, .3, 0.)), inverse(mount))
        self.assertAlmostEqual(rotated[2], 0.)
        self.assertAlmostEqual(rotated[0], original[0])
        self.assertAlmostEqual(rotated[1], original[1])

    def test_stop_defers_heading_slew_and_rollback_clears_hold(self):
        self.feedback(100.)
        q, _ = self.imu(100.01, 10.)
        self.core.calibrated, self.core.offset = True, math.radians(30.)
        q, _ = self.imu(100.02, 20.)
        self.assertAlmostEqual(rpy(q)[2], math.radians(10.))
        self.assertEqual(self.core.applied_offset, 0.)
        self.core.observe_time(50.)
        q, gyro = self.imu(50.01, -40.)
        self.assertAlmostEqual(rpy(q)[2], math.radians(-40.))
        self.assertEqual(gyro[2], .3)
        self.assertFalse(self.core.yaw_hold.held)

    def test_disabled_preserves_original_gyro_and_orientation(self):
        self.core = HeadingCalibration(policy(), encoder_yaw_hold=dict(self.hold_policy, enabled=False))
        self.feedback(100.)
        self.imu(100.01, 10.)
        q, gyro = self.imu(100.02, 20.)
        self.assertEqual(q, quaternion(orientation(yaw=20.)))
        self.assertEqual(gyro, (.1, .2, .3))

    def test_configuration_rejects_invalid_bounds(self):
        for key, value in [('enabled', 'true'), ('max_feedback_age_sec', 0.),
                           ('max_feedback_age_sec', math.nan), ('history_max_samples', 1),
                           ('history_max_samples', 2.5)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                HeadingCalibration(policy(), encoder_yaw_hold=dict(self.hold_policy, **{key: value}))


if __name__ == '__main__':
    unittest.main()
