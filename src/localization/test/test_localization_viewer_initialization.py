#!/usr/bin/env python3
"""Map selection projection and historical-vs-map display contract regression."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


spec = importlib.util.spec_from_file_location(
    'viewer_initialization', Path(__file__).resolve().parents[1]/'scripts/localization_viewer.py')
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


class ProjectionTest(unittest.TestCase):
    def test_center_and_enu_axes(self):
        project = viewer.topdown_screen_to_map
        self.assertEqual(project(400, 300, 800, 600, 10, 37, -15), (37, -15))
        self.assertEqual(project(500, 200, 800, 600, 10, 37, -15), (47, -5))

    def test_rotated_camera_uses_map_direction(self):
        np.testing.assert_allclose(
            viewer.topdown_screen_to_map(500, 300, 800, 600, 10, 0, 0, math.pi/2),
            (0, 10), atol=1e-12)

    def test_high_dpi_and_rviz_even_width_rounding(self):
        self.assertEqual(viewer.topdown_screen_to_map(500, 200, 800, 600, 10, 0, 0,
                                                     pixel_ratio=2), (20, 20))
        self.assertEqual(viewer.topdown_screen_to_map(400, 300, 801, 600, 10, 0, 0),
                         (-.1, 0))

    def test_invalid_projection_cannot_become_map_pose(self):
        for scale in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                viewer.topdown_screen_to_map(0, 0, 800, 600, scale, 0, 0)

    def test_atomic_manual_payload_and_rejected_preview(self):
        match = dict(accepted=True, route='1_right', x=10., y=20., yaw=2.8)
        result = viewer.manual_initialization_request(match, 123.)
        self.assertEqual(json.loads(json.dumps(result)), dict(frame_id='map', stamp=123.,
                         route='1_right', x=10., y=20., yaw=2.8))
        for rejected in (dict(match, accepted=False), dict(match, yaw=math.nan), None):
            with self.assertRaises(ValueError):
                viewer.manual_initialization_request(rejected, 123.)
        with self.assertRaises(ValueError):
            viewer.manual_initialization_request(match, 0.)


class SelectionRenderingTest(unittest.TestCase):
    def test_manual_payload_preserves_selected_branch(self):
        match = dict(accepted=True, route='cross', index=3, x=0., y=0., yaw=-math.pi/2)
        self.assertEqual(viewer.manual_initialization_request(match, 123.)['index'], 3)
        for index in (True, -1, 3.0, '3'):
            with self.assertRaises(ValueError):
                viewer.manual_initialization_request(dict(match, index=index), 123.)

    def test_hover_and_seek_delete_only_disappearing_markers(self):
        from visualization_msgs.msg import Marker
        route = Marker(ns='rddf/cross', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
        preview = Marker(ns='initialization_preview', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
        vehicle = Marker(ns='track/local', id=1, type=Marker.LINE_STRIP, action=Marker.ADD)
        updates, keys = viewer.scene_marker_updates([route, vehicle], set())
        self.assertEqual(len(updates), 2)
        # Entering/leaving a route must preserve the map and vehicle identities.
        updates, keys = viewer.scene_marker_updates([route, vehicle, preview], keys)
        self.assertTrue(all(marker.action == Marker.ADD for marker in updates))
        updates, keys = viewer.scene_marker_updates([route, vehicle], keys)
        self.assertEqual([(m.ns, m.id) for m in updates if m.action == Marker.DELETE],
                         [('initialization_preview', 0)])
        # Rewinding before an odometry sample removes the old vehicle, too.
        updates, keys = viewer.scene_marker_updates([route], keys)
        self.assertEqual([(m.ns, m.id) for m in updates if m.action == Marker.DELETE],
                         [('initialization_preview', 0), ('track/local', 1)])
        self.assertNotIn(Marker.DELETEALL, [m.action for m in updates])
        updates, keys = viewer.scene_marker_updates([route], keys)
        self.assertEqual([(m.ns, m.id) for m in updates if m.action == Marker.DELETE],
                         [('initialization_preview', 0), ('track/local', 1)])
        self.assertEqual([m for m in updates if m.action == Marker.ADD], [route])


class MapFrameTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root/'yongin_route_project.json').write_text(json.dumps({'origin': {'lat': 37., 'lng': 127.}}))
        (self.root/'route.csv').write_text(
            'route_id,route_name,closed,index,latitude,longitude,east_m,north_m,distance_m,path_yaw_rad\n'
            '1,test,0,0,37,127,0,0,0,0\n1,test,0,1,37,127,10,10,14,0\n')
        self.odom = Odometry()
        self.odom.pose.pose.position.x = 42.
        self.odom.pose.pose.position.y = -18.
        self.odom.pose.pose.orientation.w = 1.
        self.gps = NavSatFix()
        self.gps.status.status = 0
        self.gps.latitude, self.gps.longitude = 37., 127.

    def tearDown(self):
        self.tmp.cleanup()

    def test_rddf_map_keeps_initialized_coordinates_without_gps(self):
        model = viewer.SceneData(self.root, frame_mode='rddf_map')
        model.ingest('global', self.odom, 100, True)
        np.testing.assert_allclose(model.points('global'), [[42, -18, 0]])
        model.ingest('gps', self.gps, 101, True)
        model.ingest('local', self.odom, 102, True)
        np.testing.assert_allclose(model.points('global'), [[42, -18, 0]])
        self.assertEqual(model.summary['common_xy_translation_m'], [0., 0.])

    def test_clock_reset_preserves_map_frame_not_old_gps_anchor(self):
        model = viewer.SceneData(self.root, frame_mode='rddf_map')
        model.ingest('global', self.odom, 100, True)
        model.ingest('global', self.odom, 90, True)
        np.testing.assert_allclose(model.points('global'), [[42, -18, 0]])
        self.assertEqual(model.summary['clock_resets'], 1)
        self.assertEqual(model.summary['common_xy_translation_m'], [0., 0.])

    def test_explicit_recorded_map_mode_does_not_reanchor_new_outputs(self):
        path = self.root/'initialized_output.bag'
        with rosbag.Bag(str(path), 'w') as bag:
            for index, (key, message) in enumerate((('local', self.odom), ('gps', self.gps),
                                                    ('global', self.odom))):
                bag.write(viewer.TOPICS[key], message, rospy.Time.from_sec(100.+index))
        historical = viewer.load_data(path, None, self.root, 100)
        initialized = viewer.load_data(path, None, self.root, 100, frame_mode='rddf_map')
        np.testing.assert_allclose(historical.points('global'), [[0, 0, 0]])
        np.testing.assert_allclose(initialized.points('global'), [[42, -18, 0]])


if __name__ == '__main__':
    unittest.main()
