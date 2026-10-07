#!/usr/bin/env python3
"""Public route identification rejects stale, ambiguous and wrong-frame inputs."""
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import rospy
from mando_localization.msg import RddfMatch
from rddf_initialization_core import RddfRouteMap
from rddf_tracking_core import RddfTracker, current_rddf_match, current_rddf_members
from rddf_tracker_node import match_message


class TrackingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        (root/'yongin_route_project.json').write_text(json.dumps({'origin': {'lat': 37., 'lng': 127.}}))
        header = 'route_id,route_name,closed,index,latitude,longitude,east_m,north_m,distance_m,path_yaw_rad\n'
        for name, left, right in [('main', 0, 10), ('branch_a', 20, 30), ('branch_b', 20, 30)]:
            (root/(name+'.csv')).write_text(header+
                f'1,{name},0,0,37,127,{left},0,0,0\n1,{name},0,1,37,127,{right},0,10,0\n')
        self.tracker = RddfTracker(RddfRouteMap(root))

    def tearDown(self):
        self.tmp.cleanup()

    def test_initialization_resolves_join_without_pinning_later_route(self):
        self.configure_routes({'2': [(-10, 0), (0, 0)],
                               '3_s-static-obstacle': [(0, 0), (10, 0)]})
        self.tracker.update_pose(.02, .02, 10., 10., 'map', 0.)
        self.tracker.update_valid(True, 10.)
        self.assertFalse(self.tracker.evaluate(10.)['accepted'])
        self.tracker.initialized_source = '3_s-static-obstacle'
        result = self.tracker.evaluate(10.)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['source_route'], '3_s-static-obstacle')
        self.assertEqual(result['reason'], 'MATCHED_BY_INITIALIZATION')
        self.assertIsNone(self.tracker.initialized_source)

    def test_initialization_does_not_override_invalid_localization(self):
        self.tracker.initialized_source = 'branch_a'
        self.tracker.update_pose(25., 0., 10., 10., 'map', 0.)
        self.tracker.update_valid(False, 10.)
        self.assertEqual(self.tracker.evaluate(10.)['reason'], 'LOCALIZATION_INVALID')

    def configure_groups(self, groups, directions=None):
        root = Path(self.tmp.name)
        project = root/'yongin_route_project.json'
        data = json.loads(project.read_text())
        data['route_groups'] = groups
        data['route_directions'] = directions or {}
        project.write_text(json.dumps(data))
        self.tracker = RddfTracker(RddfRouteMap(root))

    def configure_routes(self, routes):
        root = Path(self.tmp.name)
        for path in root.glob('*.csv'):
            path.unlink()
        header = ('route_id,route_name,closed,index,latitude,longitude,'
                  'east_m,north_m,distance_m,path_yaw_rad\n')
        for route_id, (name, points) in enumerate(routes.items(), 1):
            rows, distance = [], 0.0
            for index, (x, y) in enumerate(points):
                if index:
                    previous = points[index-1]
                    distance += math.hypot(x-previous[0], y-previous[1])
                rows.append('{},{},0,{},37,127,{},{},{},0\n'.format(
                    route_id, name, index, x, y, distance))
            (root/(name+'.csv')).write_text(header+''.join(rows))
        project = root/'yongin_route_project.json'
        data = json.loads(project.read_text())
        data['route_groups'] = {}
        data['route_directions'] = {}
        project.write_text(json.dumps(data))
        self.tracker = RddfTracker(RddfRouteMap(root))

    def test_in_out_overlap_identifies_group_and_preserves_source_geometry(self):
        self.configure_groups({'parking_left': ['branch_a', 'branch_b']},
                              {'branch_b': 'reverse'})
        routes = self.tracker.route_map
        # Startup still distinguishes body headings; only current group identity
        # treats the two parking legs as one RDDF.
        self.assertFalse(routes.match(25., 1., 5.)['accepted'])
        self.feed(x=25.)
        message = self.message()
        encoded = io.BytesIO()
        message.serialize(encoded)
        message = RddfMatch().deserialize(encoded.getvalue())
        self.assertTrue(message.matched)
        self.assertEqual(message.route_name, 'parking_left')
        self.assertEqual(message.source_route_name, message.nearest.source_route_name)
        self.assertEqual({c.route_name for c in message.candidates}, {'parking_left'})
        self.assertEqual({c.source_route_name for c in message.candidates}, {'branch_a', 'branch_b'})
        self.assertEqual(message.nearest.distance_m, 1.)
        self.assertAlmostEqual(abs(message.candidates[0].heading_rad-message.candidates[1].heading_rad), math.pi)
        self.assertEqual(current_rddf_members(routes, ['parking_left']), ['branch_a', 'branch_b'])

    def test_body_heading_selects_source_leg_inside_parking_group(self):
        self.configure_groups({'parking_left': ['branch_a', 'branch_b']},
                              {'branch_b': 'reverse'})
        self.feed(x=25., yaw=0.)
        self.assertEqual(self.message().source_route_name, 'branch_a')
        self.tracker = RddfTracker(self.tracker.route_map)
        self.feed(x=25., yaw=math.pi)
        self.assertEqual(self.message().source_route_name, 'branch_b')

    def test_left_right_overlap_stays_ambiguous(self):
        self.configure_groups({'parking_left': ['branch_a'], 'parking_right': ['branch_b']})
        self.feed(x=25.)
        message = self.message()
        self.assertFalse(message.matched)
        self.assertEqual(message.reason, 'AMBIGUOUS_ROUTE')
        self.assertEqual(message.route_name, '')
        self.assertEqual(message.source_route_name, '')
        self.assertEqual({c.route_name for c in message.candidates}, {'parking_left', 'parking_right'})

    def test_invalid_group_metadata_is_rejected(self):
        for groups in ([], {'parking': ['missing']}, {'main': ['branch_a']},
                       {'parking': ['branch_a', 'branch_a']},
                       {'left': ['branch_a'], 'right': ['branch_a']},
                       {'parking': []}, {'parking': 'branch_a'}):
            with self.subTest(groups=groups), self.assertRaises(ValueError):
                self.configure_groups(groups)

    def test_all_four_real_parking_groups_cover_both_legs(self):
        routes = RddfRouteMap(Path(__file__).resolve().parents[1]/'rddf')
        expected = {
            'T_left': ['5_T-left-in', '6-T-left-out'],
            'T_right': ['5_T-right-in', '6_T-right-out'],
            'parallel_left': ['10_parallel-left-in', '11-parallel-left-out'],
            'parallel_right': ['10_parallel-right-in', '11_parallel-right-out'],
        }
        self.assertEqual(routes.route_groups, expected)
        self.assertEqual(len({routes.route_group_names.get(name, name) for name in routes.routes}), 15)
        for group, members in expected.items():
            for source in members:
                matches = [current_rddf_match(routes, point, True) for point in routes.routes[source]]
                accepted = [r for r in matches if r['accepted'] and r['route'] == group
                            and r['source_route'] == source]
                self.assertTrue(accepted, source)
                result = accepted[len(accepted)//2]
                original = routes.match(result['x'], result['y'], 5.,
                                        route_name=source, segment_index=result['index'])
                self.assertAlmostEqual(result['yaw'], original['yaw'])
                self.assertAlmostEqual(result['distance'], 0.)

    def feed(self, x=5., y=1., yaw=None, stamp=100., frame='map', valid=True):
        self.tracker.update_pose(x, y, stamp, 100., frame, yaw)
        self.tracker.update_valid(valid, 100.)

    def message(self, now=100.):
        pose = self.tracker.pose
        return match_message(self.tracker.evaluate(now), rospy.Time.from_sec(now),
            rospy.Time.from_sec(pose['stamp']) if pose else rospy.Time(), 'map')

    def test_success_message_round_trip_and_projection(self):
        self.feed()
        message = self.message()
        encoded = io.BytesIO()
        message.serialize(encoded)
        decoded = RddfMatch().deserialize(encoded.getvalue())
        self.assertTrue(decoded.matched)
        self.assertEqual((decoded.route_name, decoded.segment_index), ('main', 0))
        self.assertEqual(decoded.header.frame_id, 'map')
        self.assertEqual(decoded.pose_stamp.to_sec(), 100.)
        self.assertEqual(decoded.nearest.distance_m, 1.)
        self.assertEqual(decoded.nearest.projected_point.y, 0.)
        self.assertAlmostEqual(decoded.nearest.segment_fraction, .5)

    def test_overlap_and_far_leave_selected_route_empty(self):
        for x, reason, names in [(25., 'AMBIGUOUS_ROUTE', {'branch_a', 'branch_b'}),
                                 (100., 'TOO_FAR', set())]:
            self.feed(x=x)
            message = self.message()
            self.assertFalse(message.matched)
            self.assertEqual(message.reason, reason)
            self.assertEqual(message.route_name, '')
            self.assertEqual(message.segment_index, -1)
            self.assertTrue(message.has_nearest)
            self.assertEqual({c.route_name for c in message.candidates}, names)

    def test_waiting_invalid_and_stale_clear_geometry(self):
        self.assertEqual(self.message().reason, 'NO_GLOBAL')
        self.feed(valid=False)
        invalid = self.message()
        self.assertEqual(invalid.reason, 'LOCALIZATION_INVALID')
        self.assertFalse(invalid.has_nearest)
        self.feed()
        self.assertTrue(self.message(100.5).matched)
        stale = self.message(100.50001)
        self.assertEqual(stale.reason, 'STALE_GLOBAL')
        self.assertFalse(stale.has_nearest)
        self.assertEqual(stale.route_name, '')
        self.tracker.update_pose(5., 1., 101.1, 101.1, 'map')
        self.assertEqual(self.message(101.1).reason, 'STALE_VALID')

    def test_header_age_frame_and_nonfinite_inputs(self):
        for stamp, frame, x, reason in [
                (99., 'map', 5., 'STALE_GLOBAL'),
                (101., 'map', 5., 'INVALID_STAMP'),
                (0., 'map', 5., 'INVALID_STAMP'),
                (100., 'odom', 5., 'FRAME_MISMATCH'),
                (100., '', 5., 'FRAME_MISMATCH'),
                (100., 'map', math.nan, 'INVALID_INPUT')]:
            self.feed(x=x, stamp=stamp, frame=frame)
            message = self.message()
            self.assertEqual(message.reason, reason)
            self.assertFalse(message.has_nearest)
        self.feed(frame='/map')
        self.assertTrue(self.message().matched)

    def test_clock_rewind_discards_pose_and_validity(self):
        self.feed()
        self.assertTrue(self.message().matched)
        reset = self.message(90.)
        self.assertEqual(reset.reason, 'NO_GLOBAL')
        self.tracker.update_pose(5., 1., 90., 90., 'map')
        self.assertEqual(self.message(90.).reason, 'STALE_VALID')
        self.tracker.update_valid(True, 90.)
        self.assertTrue(self.message(90.).matched)

    def test_active_route_survives_crossing_and_transitions_only_to_successor(self):
        self.configure_routes({
            '1_right': [(0, 0), (10, 0)],
            '2': [(10, 0), (20, 0)],
            '7': [(5, -5), (5, 5)],
        })
        self.feed(x=2, y=0, yaw=0)
        first = self.message()
        self.assertEqual(first.source_route_name, '1_right')
        self.assertEqual(list(first.active_source_route_names), ['1_right', '2'])
        self.feed(x=5, y=0, yaw=0)
        crossing = self.message()
        self.assertTrue(crossing.matched)
        self.assertEqual(crossing.source_route_name, '1_right')
        self.feed(x=10, y=0, yaw=0)
        handoff = self.message()
        self.assertTrue(handoff.matched)
        self.assertEqual(handoff.reason, 'ROUTE_TRANSITION')
        self.assertEqual(handoff.source_route_name, '2')
        self.assertEqual(list(handoff.active_source_route_names), ['2'])

    def test_active_window_advances_current_and_next_in_order(self):
        self.configure_routes({
            '1_left': [(0, 0), (10, 0)],
            '2': [(10, 0), (20, 0)],
            '3_s-static-obstacle': [(20, 0), (30, 0)],
        })
        self.feed(x=2, y=0, yaw=0)
        self.assertEqual(list(self.message().active_source_route_names), ['1_left', '2'])
        self.feed(x=10, y=0, yaw=0)
        self.assertEqual(list(self.message().active_source_route_names),
                         ['2', '3_s-static-obstacle'])

    def test_active_route_is_retained_when_far_from_centerline(self):
        self.configure_routes({'1_right': [(0, 0), (10, 0)]})
        self.feed(x=2, y=0, yaw=0)
        self.assertTrue(self.message().matched)
        self.feed(x=2, y=20, yaw=0)
        result = self.message()
        self.assertTrue(result.matched)
        self.assertEqual(result.source_route_name, '1_right')
        self.assertEqual(result.reason, 'MATCHED_OFF_ROUTE')

    def test_parallel_route_does_not_jump_to_nearby_later_gear_leg(self):
        self.configure_routes({
            '10_parallel-left-in': [
                (0, 0), (1, 0), (2, 0), (3, 0), (4, 0),
                (1.1, .05), (0, .05),
            ],
        })
        self.tracker.initialized_source = '10_parallel-left-in'
        self.feed(x=0, y=0, yaw=0)
        self.assertEqual(self.message().nearest.segment_index, 0)

        self.feed(x=1.1, y=.05, yaw=0)
        matched = self.message()
        self.assertLessEqual(matched.nearest.segment_index, 2)

    def test_initial_crossing_uses_vehicle_heading(self):
        self.configure_routes({
            '1_right': [(0, 0), (10, 0)],
            '7': [(5, -5), (5, 5)],
        })
        self.feed(x=5, y=0, yaw=math.pi/2)
        matched = self.message()
        self.assertTrue(matched.matched)
        self.assertEqual(matched.reason, 'MATCHED_BY_HEADING')
        self.assertEqual(matched.source_route_name, '7')

    def test_requested_section_one_branch_overrides_initial_match(self):
        self.configure_routes({
            '1_left': [(0, 0), (10, 0)],
            '1_right': [(0, 0), (10, 0)],
        })
        self.tracker.update_successor_request('1_right')
        self.feed(x=2, y=0, yaw=0)
        matched = self.message()
        self.assertTrue(matched.matched)
        self.assertEqual(matched.reason, 'MATCHED_BY_REQUEST')
        self.assertEqual(matched.source_route_name, '1_right')

    def test_late_section_one_request_replaces_initial_heading_choice(self):
        self.configure_routes({
            '1_left': [(0, 0), (10, 0)],
            '1_right': [(10, 0), (0, 0)],
        })
        self.feed(x=2, y=0, yaw=0)
        self.assertEqual(self.message().source_route_name, '1_left')
        self.tracker.update_successor_request('1_right')
        self.feed(x=2, y=0, yaw=0)
        matched = self.message()
        self.assertEqual(matched.reason, 'MATCHED_BY_REQUEST')
        self.assertEqual(matched.source_route_name, '1_right')

    def test_initial_acquisition_still_allows_arbitrary_section(self):
        self.configure_routes({
            '1_right': [(0, 0), (10, 0)],
            '2': [(10, 0), (20, 0)],
        })
        self.feed(x=15, y=0, yaw=0)
        self.assertEqual(self.message().source_route_name, '2')

    def test_requested_branch_is_validated_and_activated_by_localization(self):
        self.configure_routes({
            '12': [(0, 0), (10, 0)],
            '13_left': [(5, 0), (5, 5)],
            '13_right': [(10, 0), (20, 0)],
        })
        self.feed(x=5, y=0, yaw=0)
        self.assertEqual(self.message().source_route_name, '12')
        self.tracker.update_successor_request('13_left')
        self.feed(x=5, y=0, yaw=0)
        selected = self.message()
        self.assertEqual(selected.reason, 'ROUTE_TRANSITION')
        self.assertEqual(selected.source_route_name, '13_left')


if __name__ == '__main__':
    unittest.main()
