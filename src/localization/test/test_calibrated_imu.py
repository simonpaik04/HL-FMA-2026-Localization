#!/usr/bin/python3
"""GNSS heading alignment regressions using independent, deterministic motion traces."""
import calendar
from dataclasses import FrozenInstanceError, replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from calibrated_imu_core import (GnssSample, HeadingCalibration, navpvt_stamp,
                                 quaternion, rpy, slerp, wrap)


def orientation(roll=0., pitch=0., yaw=0.):
    """Independent ZYX Euler construction; arguments in degrees."""
    roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def policy(**changes):
    with (PACKAGE / 'config/imu_heading_calibration.yaml').open() as stream:
        values = yaml.safe_load(stream)['imu_heading_calibration']
    # Shorter deterministic fixtures retain the production quality thresholds.
    values.update(direction_mode='forward_start', one_shot=True,
                  min_window_sec=1., min_samples=4, min_displacement_m=1.)
    values.update(changes)
    return values


def fix(stamp=100., course=0., distance=0., **changes):
    radians = math.radians(course)
    latitude = 37. + math.degrees(distance*math.sin(radians)/6378137.)
    longitude = 127. + math.degrees(distance*math.cos(radians)/
                                   (6378137.*math.cos(math.radians(37.))))
    values = dict(stamp=stamp, course=radians, speed=2.,
                  heading_accuracy=math.radians(2.), horizontal_accuracy=.2,
                  speed_accuracy=.1, latitude=latitude, longitude=longitude,
                  satellites=12, fix_ok=True, velocity_course=radians)
    values.update(changes)
    return GnssSample(**values)


def navpvt(**changes):
    values = dict(valid=7, flags2=32, year=2026, month=9, day=8,
                  hour=4, min=5, sec=6, nano=123000000, tAcc=50000, flags=1,
                  fixType=3, heading=9000000, gSpeed=2400, headAcc=250000,
                  hAcc=350, sAcc=125, lat=370000000, lon=1270000000,
                  numSV=15, velN=0, velE=2400)
    values.update(changes)
    return SimpleNamespace(**values)


class QuaternionAndUtcTest(unittest.TestCase):
    def assertAngle(self, actual, expected, places=8):
        self.assertAlmostEqual(wrap(actual-expected), 0., places=places)

    def test_slerp_takes_short_arc_and_accepts_opposite_quaternion_signs(self):
        midpoint = slerp(orientation(yaw=179.), orientation(yaw=-179.), .5)
        self.assertAngle(rpy(midpoint)[2], math.pi)
        q = orientation(5., 8., 120.)
        midpoint = slerp(q, tuple(-value for value in q), .5)
        self.assertAlmostEqual(abs(sum(a*b for a, b in zip(q, midpoint))), 1.)

    def test_quaternion_rejects_nonfinite_zero_and_large_norm_errors(self):
        for q in ((0., 0., 0., 0.), (0., 0., 0., 2.),
                  (0., 0., math.nan, 1.), (0., 0., 1.), (math.inf, 0., 0., 1.)):
            with self.subTest(q=q), self.assertRaises(ValueError):
                quaternion(q)
        self.assertEqual(quaternion((0., 0., 0., 1.0001)), (0., 0., 0., 1.))

    def test_navpvt_uses_utc_and_converts_north_clockwise_to_enu(self):
        original = navpvt()
        sample = GnssSample.from_navpvt(original)
        expected = calendar.timegm((2026, 9, 8, 4, 5, 6))+.123
        self.assertAlmostEqual(sample.stamp, expected, places=6)
        self.assertAngle(sample.course, 0.)
        self.assertAngle(sample.velocity_course, 0.)
        self.assertAlmostEqual(sample.speed, 2.4)
        self.assertAlmostEqual(sample.heading_accuracy, math.radians(2.5))
        self.assertAlmostEqual(sample.horizontal_accuracy, .350)
        self.assertAlmostEqual(sample.speed_accuracy, .125)
        self.assertAlmostEqual(sample.time_accuracy, .00005)
        north = GnssSample.from_navpvt(navpvt(heading=0, velN=2400, velE=0))
        self.assertAngle(north.course, math.pi/2)
        self.assertAngle(north.velocity_course, math.pi/2)
        self.assertEqual(original.nano, 123000000)
        with self.assertRaises(FrozenInstanceError):
            sample.course = 1.

    def test_navpvt_invalid_utc_never_falls_back_to_receipt_time(self):
        for changes in (dict(valid=3), dict(valid=0), dict(flags2=0),
                        dict(year=1969), dict(month=13), dict(day=32),
                        dict(year=2025, month=2, day=29), dict(hour=24),
                        dict(min=60), dict(sec=61), dict(nano=1000000001),
                        dict(year=2106, month=12, day=1)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                navpvt_stamp(navpvt(**changes))

    def test_navpvt_negative_fraction_and_leap_second_normalization(self):
        whole = navpvt_stamp(navpvt(nano=0))
        # Epoch-valued Python floats have sub-microsecond, not nanosecond resolution.
        self.assertAlmostEqual(navpvt_stamp(navpvt(nano=-250000000)), whole-.25, places=6)
        last = navpvt_stamp(navpvt(sec=59, nano=0))
        self.assertEqual(navpvt_stamp(navpvt(sec=60, nano=0)), last+1.)


class InitialHeadingTest(unittest.TestCase):
    def setUp(self):
        self.target = math.radians(161.47)
        self.initial = dict(yaw_rad=self.target, standard_deviation_deg=10.,
                            source='RDDF test start')
        self.core = HeadingCalibration(policy(), initial_heading=self.initial)

    def start(self, yaw=-70., roll=0., pitch=0., mount=None, stamp=100.):
        q = orientation(roll, pitch, yaw)
        self.assertTrue(self.core.observe_imu(stamp, q, (0., 0., 0.), stamp))
        self.assertTrue(self.core.initialize_heading(mount or orientation()))
        return q, self.core.step_output_orientation(q, stamp)

    def test_first_valid_output_uses_start_direction_before_gnss(self):
        q, output = self.start(roll=5., pitch=8.)
        roll, pitch, yaw = rpy(output)
        self.assertAlmostEqual(wrap(yaw-self.target), 0.)
        self.assertAlmostEqual(roll, math.radians(5.))
        self.assertAlmostEqual(pitch, math.radians(8.))
        self.assertFalse(self.core.calibrated)
        self.assertEqual(self.core.correction_count, 0)
        self.assertEqual(self.core.status()['state'], 'RDDF_INITIALIZED')
        self.assertGreaterEqual(self.core.added_yaw_variance(100.), math.radians(10.)**2)

    def test_mount_applied_once_and_real_turns_are_preserved(self):
        self.start(yaw=-50., mount=orientation(yaw=20.))
        self.assertFalse(self.core.initialize_heading(orientation(yaw=20.)))
        output = self.core.step_output_orientation(orientation(yaw=-20.), 101.)
        body_yaw = wrap(rpy(output)[2]-math.radians(20.))
        self.assertAlmostEqual(wrap(body_yaw-self.target-math.radians(30.)), 0.)
        self.core.reject('CLOCK_NOT_READY')
        self.assertAlmostEqual(wrap(rpy(self.core.output_orientation(orientation(yaw=-20.)))[2]
                                   -rpy(output)[2]), 0.)

    def test_invalid_imu_does_not_initialize_and_valid_output_waits(self):
        self.assertFalse(self.core.initialize_heading(orientation()))
        self.assertFalse(self.core.observe_imu(100., (0., 0., 0., 0.), (0., 0., 0.), 100.))
        self.assertFalse(self.core.initialize_heading(orientation()))
        with self.assertRaises(ValueError):
            self.core.step_output_orientation(orientation(), 100.)
        self.core.observe_imu(100., orientation(pitch=80.), (0., 0., 0.), 100.)
        self.assertFalse(self.core.initialize_heading(orientation()))

    def test_gnss_can_refine_initial_heading_once_with_existing_slew_limit(self):
        q, output = self.start()
        self.core.step_output_orientation(q, 100.01)
        for i in range(7):
            stamp = 101.+.2*i
            self.core.observe_imu(stamp, q, (0., 0., 0.), stamp)
            self.core.observe_twist(stamp, 2., stamp)
            self.core.observe_gnss(fix(stamp, course=150., distance=.4*i), stamp, True)
        self.assertTrue(self.core.calibrated, self.core.reason)
        before = self.core.applied_offset
        self.core.step_output_orientation(q, 102.21)
        limit = math.radians(self.core.p['max_correction_rate_degps'])*self.core.p['max_imu_gap_sec']
        self.assertLessEqual(abs(wrap(self.core.applied_offset-before)), limit+1e-9)
        for i in range(100):
            output = self.core.step_output_orientation(q, 102.22+.01*i)
        self.assertAlmostEqual(wrap(rpy(output)[2]-math.radians(150.)), 0.)
        self.assertEqual(self.core.correction_count, 1)

    def test_clock_rollback_reinitializes_from_new_session_imu(self):
        self.start()
        self.core.observe_time(50.)
        self.assertEqual(self.core.status()['state'], 'WAITING_FOR_INITIAL_HEADING')
        _, output = self.start(yaw=90., stamp=50.)
        self.assertAlmostEqual(wrap(rpy(output)[2]-self.target), 0.)

    def test_initial_heading_configuration_rejects_invalid_numbers(self):
        for key, value in (('yaw_rad', math.nan), ('yaw_rad', True),
                           ('standard_deviation_deg', 0.), ('standard_deviation_deg', math.inf)):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                HeadingCalibration(policy(), initial_heading=dict(self.initial, **{key: value}))


class HeadingCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.core = HeadingCalibration(policy())

    def aligned(self, sample, yaw=-70., speed=2., gyro=(0., 0., 0.),
                roll=0., pitch=0., clock_ready=True, mount=(0., 0., 0., 1.)):
        now = sample.stamp+.03
        self.assertTrue(self.core.observe_imu(sample.stamp-.01,
                        orientation(roll, pitch, yaw), gyro, now))
        self.assertTrue(self.core.observe_imu(sample.stamp+.01,
                        orientation(roll, pitch, yaw), gyro, now))
        self.assertTrue(self.core.observe_twist(sample.stamp-.02, speed, now))
        return self.core.observe_gnss(sample, now, clock_ready, mount)

    def calibrate(self, course=0., yaw=-70., start=100., speed=2.,
                  roll=0., pitch=0., mount=(0., 0., 0., 1.)):
        outcome = None
        for index in range(7):
            elapsed = .2*index
            outcome = self.aligned(fix(start+elapsed, course, 2.*elapsed),
                                   yaw=yaw, speed=speed, roll=roll, pitch=pitch,
                                   mount=mount)
        self.assertTrue(self.core.calibrated, outcome)
        return outcome

    def test_before_approval_output_passes_through_without_guessing_offset(self):
        q = orientation(5., 8., -70.)
        self.assertEqual(self.core.output_orientation(q), q)
        self.assertEqual(self.core.added_yaw_variance(100.), 0.)
        self.assertEqual(self.aligned(fix()), 'COLLECTING_STRAIGHT_MOTION')
        self.assertFalse(self.core.calibrated)
        self.assertEqual(self.core.output_orientation(q), q)

    def test_initial_alignment_is_world_z_rotation_with_roll_pitch_preserved(self):
        self.calibrate(roll=5., pitch=8.)
        self.assertAlmostEqual(math.degrees(self.core.offset), 70., places=7)
        corrected = self.core.output_orientation(orientation(5., 8., -70.))
        roll, pitch, yaw = map(math.degrees, rpy(corrected))
        self.assertAlmostEqual(roll, 5., places=7)
        self.assertAlmostEqual(pitch, 8., places=7)
        self.assertAlmostEqual(yaw, 0., places=7)

    def test_known_mount_yaw_is_removed_once_when_comparing_body_course(self):
        self.calibrate(yaw=-50., mount=orientation(yaw=20.))
        self.assertAlmostEqual(math.degrees(self.core.offset), 70., places=7)
        # Output remains imu_link orientation; physical mounting is owned by TF.
        yaw = rpy(self.core.output_orientation(orientation(yaw=-50.)))[2]
        self.assertAlmostEqual(math.degrees(yaw), 20., places=7)

    def test_one_shot_holds_through_stop_gps_dropout_and_clock_not_ready(self):
        self.calibrate()
        offset, stamp = self.core.offset, self.core.calibration_stamp
        bad = fix(105., course=90., speed=0., fix_ok=False)
        self.assertEqual(self.core.observe_gnss(bad, 105., False),
                         'HOLDING_CALIBRATION')
        self.assertEqual(self.core.offset, offset)
        self.assertEqual(self.core.calibration_stamp, stamp)
        self.assertEqual(self.core.correction_count, 1)
        self.assertAlmostEqual(math.degrees(rpy(self.core.output_orientation(
                               orientation(yaw=-60.)))[2]), 10., places=7)
        # A held offset intentionally preserves a subsequent raw AHRS drift.
        self.assertAlmostEqual(math.degrees(rpy(self.core.output_orientation(
                               orientation(yaw=10.)))[2]), 80., places=7)

    def test_heading_uncertainty_is_positive_grows_while_held_and_is_bounded(self):
        self.calibrate()
        stamp = self.core.calibration_stamp
        floor = math.radians(self.core.p['heading_stddev_floor_deg'])**2
        self.assertGreaterEqual(self.core.added_yaw_variance(stamp), floor)
        self.assertGreater(self.core.added_yaw_variance(stamp+30.),
                           self.core.added_yaw_variance(stamp))
        self.assertLessEqual(self.core.added_yaw_variance(stamp+1e6), math.pi**2)
        self.assertEqual(self.core.added_yaw_variance(stamp-1.),
                         self.core.added_yaw_variance(stamp))

    def test_incomplete_alignment_reports_remaining_yaw_error_as_uncertainty(self):
        self.calibrate()
        stamp = self.core.calibration_stamp
        initial = self.core.added_yaw_variance(stamp)
        self.assertGreaterEqual(initial, self.core.offset**2)
        self.core.step_output_orientation(orientation(yaw=-70.), stamp)
        for index in range(1, 2501):
            self.core.step_output_orientation(orientation(yaw=-70.), stamp+.01*index)
        self.assertLess(self.core.added_yaw_variance(stamp+25.), initial)
        self.assertGreater(self.core.added_yaw_variance(stamp+25.), 0.)

    def test_slew_caps_first_change_and_reaches_target_without_overshoot(self):
        q = orientation(5., 8., -70.)
        self.assertEqual(self.core.step_output_orientation(q, 99.), q)
        self.calibrate(roll=5., pitch=8.)
        self.assertEqual(self.core.status()['state'], 'ALIGNING')
        previous = 0.
        rate = math.radians(self.core.p['max_correction_rate_degps'])
        # The first post-approval gap is larger than an IMU sample interval.
        first = 102.
        for index in range(2500):
            result = self.core.step_output_orientation(q, first+index*.01)
            current = self.core.applied_offset
            limit = rate*(self.core.p['max_imu_gap_sec'] if index == 0 else .01)
            self.assertLessEqual(abs(wrap(current-previous)), limit+1e-10)
            self.assertLessEqual(current, self.core.offset+1e-10)
            roll, pitch, _ = map(math.degrees, rpy(result))
            self.assertAlmostEqual(roll, 5., places=7)
            self.assertAlmostEqual(pitch, 8., places=7)
            previous = current
        self.assertEqual(self.core.status()['state'], 'CALIBRATED')
        self.assertAlmostEqual(self.core.applied_offset, self.core.offset)
        self.assertAlmostEqual(math.degrees(rpy(result)[2]), 0., places=7)

    def test_long_imu_dropout_does_not_apply_accumulated_correction_at_once(self):
        self.calibrate()
        q = orientation(yaw=-70.)
        first = self.core.step_output_orientation(q, 102.)
        self.assertAlmostEqual(math.degrees(rpy(first)[2]), -70., places=7)
        self.core.step_output_orientation(q, 132.)
        maximum = self.core.p['max_correction_rate_degps']*self.core.p['max_imu_gap_sec']
        self.assertAlmostEqual(math.degrees(self.core.applied_offset), maximum, places=7)

    def test_slew_uses_short_offset_across_minus_plus_180_boundary(self):
        self.calibrate(course=179., yaw=-179.)
        self.assertAlmostEqual(math.degrees(self.core.offset), -2., places=7)
        q = orientation(yaw=-179.)
        self.core.step_output_orientation(q, 102.)
        for index in range(1, 101):
            result = self.core.step_output_orientation(q, 102.+index*.01)
        self.assertAlmostEqual(math.degrees(self.core.applied_offset), -2., places=7)
        self.assertAlmostEqual(math.degrees(rpy(result)[2]), 179., places=7)

    def test_clock_gate_and_explicit_disabled_mode_block_new_alignment(self):
        self.assertEqual(self.aligned(fix(), clock_ready=False), 'CLOCK_NOT_READY')
        self.assertFalse(self.core.calibrated)
        self.core = HeadingCalibration(policy(enabled=False))
        self.assertEqual(self.aligned(fix()), 'CALIBRATION_DISABLED')
        self.assertFalse(self.core.calibrated)

    def test_rewind_clears_history_and_offset_but_old_sensor_header_does_not(self):
        self.calibrate()
        self.assertFalse(self.core.observe_imu(100., orientation(), (0., 0., 0.), 102.))
        self.assertTrue(self.core.calibrated)
        self.assertTrue(self.core.observe_time(50.))
        self.assertFalse(self.core.calibrated)
        self.assertEqual(self.core.offset, 0.)
        self.assertEqual(len(self.core.imus), 0)
        self.assertEqual(len(self.core.twists), 0)
        self.assertIsNone(self.core.last_gnss_stamp)
        self.assertEqual(self.core.reason, 'CLOCK_ROLLBACK_RESET')

    def test_delayed_gnss_uses_measurement_time_slerp_and_causal_encoder(self):
        self.assertTrue(self.core.observe_twist(100.08, 2., 100.10))
        self.assertTrue(self.core.observe_imu(100.09, orientation(yaw=-71.),
                                             (0., 0., 0.), 100.12))
        self.assertTrue(self.core.observe_imu(100.11, orientation(yaw=-69.),
                                             (0., 0., 0.), 100.12))
        # Latest-at-receipt values are deliberately incompatible with the fix.
        self.assertTrue(self.core.observe_imu(100.4, orientation(yaw=20.),
                                             (0., 0., 1.), 100.4))
        self.assertTrue(self.core.observe_twist(100.4, 0., 100.4))
        result = self.core.observe_gnss(fix(100.1), 100.4, True)
        self.assertEqual(result, 'COLLECTING_STRAIGHT_MOTION')
        self.assertAlmostEqual(math.degrees(self.core.window[0][1]), 70., places=7)

    def test_missing_right_imu_bracket_waits_then_accepts_same_gnss(self):
        sample = fix(100.)
        self.core.observe_twist(99.98, 2., 100.)
        self.core.observe_imu(99.99, orientation(yaw=-70.), (0., 0., 0.), 100.)
        self.assertEqual(self.core.observe_gnss(sample, 100., True),
                         'WAITING_FOR_IMU_ALIGNMENT')
        self.assertIsNone(self.core.last_gnss_stamp)
        self.core.observe_imu(100.01, orientation(yaw=-70.), (0., 0., 0.), 100.02)
        self.assertEqual(self.core.observe_gnss(sample, 100.02, True),
                         'COLLECTING_STRAIGHT_MOTION')

    def test_imu_gap_and_future_encoder_are_not_extrapolated(self):
        self.core.observe_imu(99.8, orientation(yaw=-70.), (0., 0., 0.), 100.01)
        self.core.observe_imu(100.01, orientation(yaw=-70.), (0., 0., 0.), 100.01)
        self.core.observe_twist(99.98, 2., 100.01)
        self.assertEqual(self.core.observe_gnss(fix(), 100.01, True),
                         'WAITING_FOR_IMU_ALIGNMENT')
        self.core = HeadingCalibration(policy())
        self.core.observe_imu(100., orientation(yaw=-70.), (0., 0., 0.), 100.01)
        self.core.observe_twist(100.01, 2., 100.01)
        self.assertEqual(self.core.observe_gnss(fix(), 100.01, True),
                         'WAITING_FOR_ENCODER_ALIGNMENT')

    def test_gnss_quality_failures_never_approve_or_keep_partial_window(self):
        cases = [(dict(fix_ok=False), 'GNSS_FIX_INVALID'),
                 (dict(satellites=1), 'GNSS_FIX_INVALID'),
                 (dict(speed=0.), 'GNSS_TOO_SLOW'),
                 (dict(heading_accuracy=math.radians(45.)), 'GNSS_ACCURACY_REJECTED'),
                 (dict(horizontal_accuracy=20.), 'GNSS_ACCURACY_REJECTED'),
                 (dict(speed_accuracy=5.), 'GNSS_ACCURACY_REJECTED'),
                 (dict(time_accuracy=.051), 'GNSS_TIME_ACCURACY_REJECTED'),
                 (dict(time_accuracy=-.001), 'GNSS_TIME_ACCURACY_REJECTED'),
                 (dict(time_accuracy=math.nan), 'GNSS_NONFINITE'),
                 (dict(course=math.nan), 'GNSS_NONFINITE'),
                 (dict(latitude=91.), 'GNSS_ACCURACY_REJECTED'),
                 (dict(velocity_course=math.pi/2), 'GNSS_VELOCITY_HEADING_MISMATCH')]
        for changes, expected in cases:
            with self.subTest(changes=changes):
                self.core = HeadingCalibration(policy())
                self.aligned(fix())
                self.assertEqual(self.aligned(fix(100.2, distance=.4, **changes)), expected)
                self.assertFalse(self.core.calibrated)
                self.assertEqual(len(self.core.window), 0)

    def test_reverse_turn_tilt_and_encoder_disagreement_are_separate_rejections(self):
        cases = [(dict(speed=-2.), 'REVERSE_DURING_FORWARD_START'),
                 (dict(speed=0.), 'ENCODER_TOO_SLOW'),
                 (dict(speed=5.), 'ENCODER_GNSS_SPEED_MISMATCH'),
                 (dict(gyro=(0., 0., math.radians(10.))), 'TURNING'),
                 (dict(pitch=30.), 'TILT_TOO_LARGE')]
        for changes, expected in cases:
            with self.subTest(changes=changes):
                self.core = HeadingCalibration(policy())
                self.assertEqual(self.aligned(fix(), **changes), expected)
                self.assertFalse(self.core.calibrated)

    def test_verified_signed_encoder_can_align_reverse_motion(self):
        self.core = HeadingCalibration(policy(direction_mode='signed_encoder'))
        self.calibrate(course=180., yaw=-70., speed=-2.)
        self.assertAlmostEqual(math.degrees(self.core.offset), 70., places=7)

    def test_duplicate_gnss_cannot_satisfy_minimum_sample_count(self):
        self.aligned(fix())
        for _ in range(10):
            self.assertEqual(self.core.observe_gnss(fix(), 100.03, True),
                             'GNSS_DUPLICATE_OR_REORDERED')
        self.assertFalse(self.core.calibrated)
        self.assertEqual(len(self.core.window), 0)

    def test_straight_course_and_stable_offset_are_both_required(self):
        self.aligned(fix())
        self.assertEqual(self.aligned(fix(100.2, course=20., distance=.4), yaw=-50.),
                         'COURSE_NOT_STRAIGHT')
        self.core = HeadingCalibration(policy())
        self.aligned(fix())
        self.assertEqual(self.aligned(fix(100.2, distance=.4), yaw=-50.),
                         'OFFSET_NOT_STABLE')

    def test_position_displacement_must_agree_with_gnss_reported_course(self):
        results = []
        for index in range(7):
            elapsed = .2*index
            # Velocity reports east, but recorded coordinates move north.
            sample = replace(fix(100.+elapsed, 90., 2.*elapsed),
                             course=0., velocity_course=0.)
            results.append(self.aligned(sample))
        self.assertIn('GPS_POSITION_COURSE_MISMATCH', results)
        self.assertFalse(self.core.calibrated)

    def test_no_displacement_and_gaps_cannot_build_a_false_straight_segment(self):
        for index in range(7):
            result = self.aligned(fix(100.+.2*index))
        self.assertEqual(result, 'WAITING_FOR_DISPLACEMENT')
        self.assertFalse(self.core.calibrated)
        self.aligned(fix(103., distance=6.))
        self.assertEqual(len(self.core.window), 1)

    def test_sensor_history_is_bounded_and_rejects_invalid_inputs(self):
        self.core = HeadingCalibration(policy(history_max_samples=5))
        for index in range(20):
            stamp = 100.+index*.02
            self.assertTrue(self.core.observe_imu(stamp, orientation(),
                                                 (0., 0., 0.), stamp))
        self.assertEqual(len(self.core.imus), 5)
        self.assertFalse(self.core.observe_imu(100.4, (0., 0., 0., 0.),
                                              (0., 0., 0.), 100.4))
        self.assertFalse(self.core.observe_imu(100.4, orientation(),
                                              (0., math.nan, 0.), 100.4))
        self.assertFalse(self.core.observe_twist(100.4, math.nan, 100.4))
        self.assertFalse(self.core.observe_imu(102., orientation(), (0., 0., 0.), 100.4))

    def test_invalid_policy_cannot_silently_enable_unverified_recalibration(self):
        cases = (dict(enabled='true'), dict(min_samples=1),
                 dict(history_sec=.5), dict(direction_mode='unsigned'),
                 dict(max_heading_accuracy_deg=math.nan), dict(min_samples=2.5),
                 dict(min_window_sec=20., max_window_sec=12.))
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                HeadingCalibration(policy(**changes))


class RepeatedHeadingCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.core = HeadingCalibration(policy(one_shot=False))
        self.last_stamp = None

    def aligned(self, sample, yaw=-70., speed=2., gyro=(0., 0., 0.)):
        now = sample.stamp+.03
        self.assertTrue(self.core.observe_imu(sample.stamp-.01,
                                             orientation(yaw=yaw), gyro, now))
        self.assertTrue(self.core.observe_imu(sample.stamp+.01,
                                             orientation(yaw=yaw), gyro, now))
        self.assertTrue(self.core.observe_twist(sample.stamp-.02, speed, now))
        self.last_stamp = now
        return self.core.observe_gnss(sample, now, True)

    def window(self, start=100., course=0., yaw=-70., speed=2.):
        return [self.aligned(fix(start+.2*i, course, .4*i), yaw, speed)
                for i in range(6)]

    def settle(self, yaw=-70.):
        # Exercise the actual output slew; do not assign applied_offset directly.
        for i in range(201):
            self.core.step_output_orientation(orientation(yaw=yaw),
                                              self.last_stamp+.01*i)
        self.assertAlmostEqual(wrap(self.core.applied_offset-self.core.offset), 0.)

    def test_each_repeat_requires_a_fresh_complete_window_and_replaces_offset(self):
        self.assertEqual(self.window()[-1], 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 1)
        self.assertEqual(len(self.core.window), 0)
        self.assertAlmostEqual(math.degrees(self.core.offset), 70.)
        self.settle()

        # Ten degrees of later AHRS drift must replace 70 with 60, not add 60.
        for i in range(5):
            result = self.aligned(fix(104.+.2*i, distance=.4*i), yaw=-60.)
            self.assertEqual(result, 'COLLECTING_STRAIGHT_MOTION')
            self.assertEqual(self.core.correction_count, 1)
        self.assertEqual(self.aligned(fix(105., distance=2.), yaw=-60.), 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 2)
        self.assertEqual(len(self.core.window), 0)
        self.assertAlmostEqual(math.degrees(self.core.offset), 60.)

    def test_repeat_waits_until_previous_offset_has_been_applied(self):
        self.window()
        self.assertNotAlmostEqual(self.core.applied_offset, self.core.offset)
        self.assertEqual(self.aligned(fix(101.2, distance=2.4)),
                         'WAITING_FOR_PREVIOUS_ALIGNMENT')
        self.assertEqual(self.core.correction_count, 1)
        self.assertEqual(len(self.core.window), 0)
        self.settle()
        self.assertEqual(self.window(start=104.)[-1], 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 2)

    def test_unsigned_reverse_course_and_large_heading_mismatch_hold_last_alignment(self):
        self.window()
        self.settle()
        offset, stamp = self.core.offset, self.core.calibration_stamp
        for start, course, yaw in ((104., 180., -70.), (106., 0., -20.)):
            with self.subTest(course=course, yaw=yaw):
                # Positive encoder speed deliberately provides no reverse evidence.
                outcomes = self.window(start, course=course, yaw=yaw, speed=2.)
                self.assertEqual(set(outcomes), {'REPEAT_HEADING_DIFFERENCE_TOO_LARGE'})
                self.assertEqual(self.core.correction_count, 1)
                self.assertEqual(self.core.offset, offset)
                self.assertEqual(self.core.calibration_stamp, stamp)
                self.assertEqual(len(self.core.window), 0)

    def test_rejected_repeat_discards_partial_window_and_preserves_valid_calibration(self):
        self.window()
        self.settle()
        offset = self.core.offset
        for i in range(3):
            self.aligned(fix(104.+.2*i, distance=.4*i), yaw=-65.)
        self.assertEqual(len(self.core.window), 3)
        self.assertEqual(self.aligned(fix(104.6, distance=1.2), yaw=-10.),
                         'REPEAT_HEADING_DIFFERENCE_TOO_LARGE')
        self.assertEqual(len(self.core.window), 0)
        self.assertTrue(self.core.calibrated)
        self.assertEqual(self.core.offset, offset)
        for i in range(5):
            self.aligned(fix(104.8+.2*i, distance=1.6+.4*i), yaw=-65.)
            self.assertEqual(self.core.correction_count, 1)
        self.assertEqual(self.aligned(fix(105.8, distance=3.6), yaw=-65.), 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 2)

    def test_repeat_retains_quality_and_turning_checks(self):
        self.window()
        self.settle()
        offset = self.core.offset
        self.assertEqual(self.aligned(fix(104., fix_ok=False)), 'GNSS_FIX_INVALID')
        self.assertEqual(self.aligned(fix(104.2, speed=0.)), 'GNSS_TOO_SLOW')
        self.assertEqual(self.aligned(fix(104.4), gyro=(0., 0., math.radians(10.))),
                         'TURNING')
        self.assertTrue(self.core.calibrated)
        self.assertEqual(self.core.correction_count, 1)
        self.assertEqual(self.core.offset, offset)
        self.assertEqual(len(self.core.window), 0)

    def test_real_heading_changes_and_signed_reverse_do_not_change_fixed_offset(self):
        self.window()
        self.settle()
        self.assertEqual(self.window(104., course=90., yaw=20.)[-1], 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 2)
        self.assertAlmostEqual(math.degrees(self.core.offset), 70.)

        self.core = HeadingCalibration(policy(direction_mode='signed_encoder', one_shot=False))
        self.window(start=200.)
        self.settle()
        self.assertEqual(self.window(204., course=180., speed=-2.)[-1], 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 2)
        self.assertAlmostEqual(math.degrees(self.core.offset), 70.)

    def test_repeat_difference_wraps_across_plus_minus_180_degrees(self):
        self.window(course=179., yaw=-170.)
        self.settle(yaw=-170.)
        self.assertEqual(self.window(104., course=-179., yaw=-170.)[-1], 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 2)
        self.assertAlmostEqual(math.degrees(self.core.offset), -9.)

    def test_repeat_uses_stationary_drift_guarded_yaw_without_double_correction(self):
        self.core = HeadingCalibration(policy(one_shot=False), encoder_yaw_hold=dict(
            enabled=True, max_feedback_age_sec=.3, history_max_samples=500))
        self.window()
        self.settle()
        self.assertTrue(self.core.observe_feedback(104., 0, 0., 1))
        self.assertTrue(self.core.observe_imu(104.01, orientation(yaw=-40.),
                                             (0., 0., 0.), 104.01))
        self.assertTrue(self.core.yaw_hold.held)
        self.assertAlmostEqual(math.degrees(self.core.yaw_hold.offset), -30.)
        _, guarded_q, _ = self.core.imus[-1]
        output = self.core.step_output_orientation(guarded_q, 104.01)
        self.assertAlmostEqual(rpy(output)[2], 0.)

        for i in range(6):
            stamp = 105.+.2*i
            self.assertTrue(self.core.observe_feedback(stamp-.04, 5, 2., i+2))
            result = self.aligned(fix(stamp, distance=.4*i), yaw=-40.)
        self.assertEqual(result, 'CALIBRATED')
        self.assertEqual(self.core.correction_count, 2)
        self.assertAlmostEqual(math.degrees(self.core.offset), 70.)
        _, guarded_q, _ = self.core.imus[-1]
        output = self.core.step_output_orientation(guarded_q, self.last_stamp)
        self.assertAlmostEqual(rpy(output)[2], 0.)

    def test_repeat_threshold_must_be_strictly_between_zero_and_90_degrees(self):
        for value in (0., -1., 90., 180., math.nan, math.inf, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                HeadingCalibration(policy(one_shot=False,
                                          max_repeat_heading_difference_deg=value))


if __name__ == '__main__':
    unittest.main()
