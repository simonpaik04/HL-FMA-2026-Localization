#!/usr/bin/env python3
"""Directed RDDF snapping must not silently choose a competing road branch."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location(
    "rddf_initialization_core",
    Path(__file__).resolve().parents[1] / "scripts/rddf_initialization_core.py",
)
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)


class RddfInitializationCoreTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name)
        self.project = self.directory / "yongin_route_project.json"
        self.project.write_text(json.dumps({"origin": {"lat": 37.0, "lng": 127.0}}))

    def tearDown(self):
        self.tmp.cleanup()

    def route(self, name, points, closed=False):
        path = self.directory / (name + ".csv")
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "route_id", "route_name", "closed", "index", "latitude",
                "longitude", "east_m", "north_m", "distance_m", "path_yaw_rad",
            ])
            distance = 0.0
            for index, (x, y) in enumerate(points):
                if index:
                    distance += math.hypot(x - points[index - 1][0], y - points[index - 1][1])
                other = points[min(index + 1, len(points) - 1)]
                yaw = math.atan2(other[1] - y, other[0] - x)
                writer.writerow([name, name, str(closed).lower(), index, 37, 127,
                                 x, y, distance, yaw])
        return path

    def test_segment_projection_and_directed_heading(self):
        self.route("westbound", [(20, 10), (10, 10), (0, 10)])
        match = core.RddfRouteMap(self.directory).match(15, 12, 3)
        self.assertTrue(match["accepted"])
        self.assertEqual(match["route"], "westbound")
        self.assertEqual(match["index"], 0)
        self.assertAlmostEqual(match["x"], 15)
        self.assertAlmostEqual(match["y"], 10)
        self.assertAlmostEqual(abs(match["yaw"]), math.pi)
        self.assertAlmostEqual(match["distance"], 2)
        self.assertAlmostEqual(match["fraction"], 0.5)

    def test_reverse_heading_keeps_projection_and_index(self):
        self.route("parking", [(0, 0), (10, 0)])
        self.project.write_text(json.dumps({
            "origin": {"lat": 37, "lng": 127},
            "route_directions": {"parking": "reverse"},
        }))
        match = core.RddfRouteMap(self.directory).match(3, 1, 2)
        self.assertTrue(match["accepted"])
        self.assertAlmostEqual(abs(match["yaw"]), math.pi)
        self.assertEqual((match["x"], match["y"], match["index"]), (3, 0, 0))
        self.assertAlmostEqual(match["fraction"], .3)

    def test_invalid_direction_metadata_is_rejected(self):
        self.route("parking", [(0, 0), (10, 0)])
        for directions in ({"parking": "backward"}, {"typo": "reverse"}, []):
            with self.subTest(directions=directions):
                self.project.write_text(json.dumps({
                    "origin": {"lat": 37, "lng": 127},
                    "route_directions": directions,
                }))
                with self.assertRaises(ValueError):
                    core.RddfRouteMap(self.directory)

    def test_yongin_parking_body_headings(self):
        route_map = core.RddfRouteMap(Path(__file__).resolve().parents[1] / "rddf")
        reverse = {"5_T-left-in", "5_T-right-in", "10_parallel-left-in"}
        self.assertEqual(set(route_map.route_directions), reverse)
        for segment in route_map._segments:
            with self.subTest(route=segment["route"], index=segment["index"]):
                vector = segment["vector"]
                heading = np.array([math.cos(segment["yaw"]), math.sin(segment["yaw"])])
                alignment = float(np.dot(heading, vector) / np.linalg.norm(vector))
                self.assertAlmostEqual(alignment, -1 if segment["route"] in reverse else 1)

    def test_nearby_samples_of_one_straight_do_not_create_ambiguity(self):
        self.route("dense", [(i * 0.1, 0) for i in range(101)])
        match = core.RddfRouteMap(self.directory).match(5.025, 1, 3)
        self.assertTrue(match["accepted"])
        self.assertAlmostEqual(match["x"], 5.025)

    def test_adjacent_turn_is_one_physical_branch(self):
        self.route("corner", [(0, 0), (5, 0), (5, 5)])
        self.assertTrue(core.RddfRouteMap(self.directory).match(4.9, 0.1, 1)["accepted"])

    def test_crossing_routes_are_ambiguous_and_explicit_route_resolves(self):
        self.route("east", [(-10, 0), (10, 0)])
        self.route("north", [(0, -10), (0, 10)])
        route_map = core.RddfRouteMap(self.directory)
        match = route_map.match(0, 0, 3)
        self.assertFalse(match["accepted"])
        self.assertEqual(match["reason"], "AMBIGUOUS_ROUTE")
        self.assertEqual({c["route"] for c in match["candidates"]}, {"east", "north"})
        selected = route_map.match(0, 0, 3, route_name="north")
        self.assertTrue(selected["accepted"])
        self.assertAlmostEqual(selected["yaw"], math.pi / 2)

    def test_opposite_directions_on_identical_geometry_are_ambiguous(self):
        self.route("east", [(-10, 0), (10, 0)])
        self.route("west", [(10, 0), (-10, 0)])
        self.assertEqual(core.RddfRouteMap(self.directory).match(0, 0, 3)["reason"],
                         "AMBIGUOUS_ROUTE")

    def test_adjacent_retraced_segment_is_ambiguous(self):
        self.route("reverse", [(0, 0), (10, 0), (0, 0)])
        self.assertFalse(core.RddfRouteMap(self.directory).match(5, 0, 1)["accepted"])

    def test_self_intersection_does_not_choose_first_index(self):
        self.route("cross", [(-10, 0), (10, 0), (10, 10), (0, 10), (0, -10)])
        self.assertEqual(core.RddfRouteMap(self.directory).match(0, 0, 1)["reason"],
                         "AMBIGUOUS_ROUTE")

    def test_overlapping_same_direction_routes_are_equivalent(self):
        self.route("a", [(0, 0), (10, 0)])
        self.route("b", [(0, 0), (4, 0), (10, 0)])
        self.assertTrue(core.RddfRouteMap(self.directory).match(5, 0.2, 2)["accepted"])

    def test_manual_segment_choice_resolves_each_self_crossing_branch(self):
        self.route("cross", [(-10, 0), (10, 0), (10, 10), (0, 10), (0, -10)])
        route_map = core.RddfRouteMap(self.directory)
        ambiguous = route_map.match(0, 0, 1, route_name="cross")
        self.assertEqual(ambiguous["reason"], "AMBIGUOUS_ROUTE")
        for index, yaw in ((0, 0), (3, -math.pi / 2)):
            selected = route_map.match(0, .1, 1, route_name="cross", segment_index=index)
            self.assertTrue(selected["accepted"])
            self.assertEqual(selected["index"], index)
            self.assertAlmostEqual(selected["yaw"], yaw)
        # Explicit selection does not change automatic GPS ambiguity handling.
        self.assertFalse(route_map.match(0, 0, 1)["accepted"])

    def test_manual_segment_choice_keeps_distance_and_identity_checks(self):
        self.route("road", [(0, 0), (10, 0)])
        route_map = core.RddfRouteMap(self.directory)
        self.assertEqual(route_map.match(5, 10, 1, route_name="road", segment_index=0)["reason"],
                         "TOO_FAR")
        for index in (True, -1, 0.0, "0", 99):
            self.assertFalse(route_map.match(5, 0, 1, route_name="road", segment_index=index)["accepted"])
        self.assertFalse(route_map.match(5, 0, 1, segment_index=0)["accepted"])
        self.assertFalse(route_map.match(5, 0, 1, route_name="missing", segment_index=0)["accepted"])

    def test_all_overlap_candidates_remain_selectable(self):
        for index in range(10):
            angle = index * math.pi / 10
            x, y = 10 * math.cos(angle), 10 * math.sin(angle)
            self.route(str(index), [(-x, -y), (x, y)])
        route_map = core.RddfRouteMap(self.directory)
        match = route_map.match(0, 0, 1)
        self.assertEqual(match["reason"], "AMBIGUOUS_ROUTE")
        self.assertEqual(len(match["candidates"]), 10)
        for candidate in match["candidates"]:
            self.assertTrue(route_map.match(0, 0, 1, route_name=candidate["route"],
                                           segment_index=candidate["index"])["accepted"])

    def test_parallel_lanes_do_not_collapse_into_same_heading(self):
        self.route("left", [(0, 0), (10, 0)])
        self.route("right", [(0, 1), (10, 1)])
        route_map = core.RddfRouteMap(self.directory)
        self.assertFalse(route_map.match(5, 0.5, 2)["accepted"])
        self.assertTrue(route_map.match(5, 0.1, 2, ambiguity_distance_m=0.2)["accepted"])

    def test_far_and_nonfinite_input_rejected(self):
        self.route("road", [(0, 0), (10, 0)])
        route_map = core.RddfRouteMap(self.directory)
        self.assertEqual(route_map.match(5, 20, 5)["reason"], "TOO_FAR")
        for args in [(float("nan"), 0, 5), (0, float("inf"), 5), (0, 0, -1),
                     (0, 0, float("inf")), ("bad", 0, 2), (1e308, 1e308, 5)]:
            self.assertEqual(route_map.match(*args)["reason"], "INVALID_INPUT")
        self.assertEqual(route_map.match(0, 0, 2, -1)["reason"], "INVALID_INPUT")
        self.assertEqual(route_map.match(0, 0, 2, route_name="missing")["reason"], "UNKNOWN_ROUTE")

    def test_gps_projection_uses_existing_ellipsoid_formula(self):
        self.route("road", [(0, 0), (10, 0)])
        route_map = core.RddfRouteMap(self.directory)
        self.assertEqual(route_map.project_gps(37, 127), (0.0, 0.0))
        lat = math.radians(37)
        d = math.sqrt(1 - 6.69437999014e-3 * math.sin(lat) ** 2)
        expected = (6378137.0 / d * math.cos(lat) * math.radians(0.001),
                    6378137.0 * (1 - 6.69437999014e-3) / d ** 3 * math.radians(0.001))
        np.testing.assert_allclose(route_map.project_gps(37.001, 127.001), expected,
                                   rtol=1e-10, atol=1e-8)
        for lat, lon in [(91, 127), (37, 181), (float("nan"), 0), (0, float("inf"))]:
            with self.assertRaises(ValueError):
                route_map.project_gps(lat, lon)

    def test_closed_route_adds_last_to_first_segment(self):
        self.route("triangle", [(0, 0), (10, 0), (10, 10)], closed=True)
        match = core.RddfRouteMap(self.directory).match(5, 5, 1)
        self.assertTrue(match["accepted"])
        self.assertEqual(match["index"], 2)
        self.assertAlmostEqual(match["yaw"], -3 * math.pi / 4)

    def test_degenerate_points_are_skipped_but_empty_geometry_fails(self):
        self.route("duplicate", [(0, 0), (0, 0), (10, 0)])
        self.assertTrue(core.RddfRouteMap(self.directory).match(5, 0, 1)["accepted"])
        self.route("invalid", [(0, 0), (0, 0)])
        with self.assertRaises(ValueError):
            core.RddfRouteMap(self.directory)

    def test_bad_route_data_and_bad_origin_fail_explicitly(self):
        path = self.route("road", [(0, 0), (float("nan"), 0)])
        with self.assertRaises(ValueError):
            core.RddfRouteMap(self.directory)
        path.write_text("east_m,north_m\n0,0\n10,0\n")
        with self.assertRaises(ValueError):
            core.RddfRouteMap(self.directory)
        self.route("road", [(0, 0), (10, 0)])
        self.project.write_text(json.dumps({"origin": {"lat": 100, "lng": 127}}))
        with self.assertRaises(ValueError):
            core.RddfRouteMap(self.directory)

    def test_duplicate_names_and_nonconsecutive_indices_fail(self):
        path = self.route("road", [(0, 0), (10, 0)])
        duplicate = self.directory / "duplicate.csv"
        duplicate.write_text(path.read_text())
        with self.assertRaises(ValueError):
            core.RddfRouteMap(self.directory)
        duplicate.unlink()
        path.write_text(path.read_text().replace("false,1,", "false,3,"))
        with self.assertRaises(ValueError):
            core.RddfRouteMap(self.directory)

    def test_routes_are_read_only_and_constructor_tolerances_are_checked(self):
        self.route("road", [(0, 0), (10, 0)])
        route_map = core.RddfRouteMap(self.directory)
        with self.assertRaises(ValueError):
            route_map.routes["road"][0, 0] = 12
        for kwargs in ({"same_branch_distance_m": -1}, {"same_branch_heading_deg": 180},
                       {"same_branch_distance_m": float("nan")}):
            with self.assertRaises(ValueError):
                core.RddfRouteMap(self.directory, **kwargs)


if __name__ == "__main__":
    unittest.main()
