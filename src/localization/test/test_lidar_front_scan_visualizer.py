#!/usr/bin/env python3
"""전방 표시 마스크의 장착 회전, 경계, 원본 보존을 검사한다."""

import importlib.util
import math
from pathlib import Path
import unittest

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "lidar_front_scan_visualizer.py"
SPEC = importlib.util.spec_from_file_location("lidar_front_scan_visualizer", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FrontScanMaskTest(unittest.TestCase):
    def scan(self):
        scan = LaserScan()
        scan.header.seq = 17
        scan.header.frame_id = "laser_link"
        scan.header.stamp.secs = 123
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 4.0
        scan.time_increment = 0.001
        scan.scan_time = 0.1
        scan.range_min, scan.range_max = 0.1, 30.0
        scan.ranges = [1.0] * 9
        scan.intensities = [12.0] * 9
        return scan

    def mask(self, scan, q):
        return MODULE.mask_front_sector(scan, q, -math.pi / 2, math.pi / 2)

    def test_upside_down_rear_facing_mount_keeps_raw_outer_angles(self):
        result = self.mask(self.scan(), Quaternion(0, 1, 0, 0))
        self.assertEqual([0, 1, 2, 6, 7, 8],
                         [i for i, r in enumerate(result.ranges) if math.isfinite(r)])
        self.assertTrue(all(math.isnan(result.ranges[i]) for i in (3, 4, 5)))
        self.assertEqual([0.0] * 3, result.intensities[3:6])

    def test_upright_front_facing_mount_keeps_raw_center_angles(self):
        result = self.mask(self.scan(), Quaternion(0, 0, 0, 1))
        self.assertEqual([2, 3, 4, 5, 6],
                         [i for i, r in enumerate(result.ranges) if math.isfinite(r)])

    def test_header_geometry_timing_and_original_are_preserved(self):
        source = self.scan()
        result = self.mask(source, Quaternion(0, 1, 0, 0))
        self.assertEqual(source.header, result.header)
        for field in ("angle_min", "angle_max", "angle_increment", "time_increment",
                      "scan_time", "range_min", "range_max"):
            self.assertEqual(getattr(source, field), getattr(result, field))
        self.assertEqual([1.0] * 9, source.ranges)
        self.assertEqual([12.0] * 9, source.intensities)
        self.assertEqual(len(source.ranges), len(result.ranges))

    def test_boundary_float32_error_and_empty_intensities(self):
        scan = self.scan()
        scan.angle_min = -math.pi / 2 - 4.4e-8
        scan.angle_increment = math.pi
        scan.ranges, scan.intensities = [1.0, 2.0], []
        result = self.mask(scan, Quaternion(0, 1, 0, 0))
        self.assertEqual([1.0, 2.0], result.ranges)
        self.assertEqual([], result.intensities)

    def test_invalid_measurements_are_not_fabricated_as_free_space(self):
        scan = self.scan()
        scan.ranges[0], scan.ranges[1] = float("nan"), float("inf")
        result = self.mask(scan, Quaternion(0, 1, 0, 0))
        self.assertTrue(math.isnan(result.ranges[0]))
        self.assertTrue(math.isinf(result.ranges[1]))
        self.assertTrue(math.isnan(result.ranges[4]))

    def test_invalid_quaternion_angles_and_intensity_shape_are_rejected(self):
        with self.assertRaises(ValueError):
            self.mask(self.scan(), Quaternion(0, 0, 0, 0))
        scan = self.scan()
        scan.angle_increment = float("nan")
        with self.assertRaises(ValueError):
            self.mask(scan, Quaternion(0, 1, 0, 0))
        scan = self.scan()
        scan.intensities = [1.0]
        with self.assertRaises(ValueError):
            self.mask(scan, Quaternion(0, 1, 0, 0))


if __name__ == "__main__":
    unittest.main()
