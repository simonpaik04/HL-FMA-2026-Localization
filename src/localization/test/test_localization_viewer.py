#!/usr/bin/env python3
"""Display invariants shared by live subscriptions and recorded results."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
import rospy
import rosbag
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

spec = importlib.util.spec_from_file_location('viewer', Path(__file__).resolve().parents[1]/'scripts/localization_viewer.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)

class ViewerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root/'yongin_route_project.json').write_text(json.dumps({'origin':{'lat':37.,'lng':127.}}))
        (self.root/'route.csv').write_text('route_id,route_name,closed,index,latitude,longitude,east_m,north_m,distance_m,path_yaw_rad\n1,test,0,0,37,127,0,0,0,0\n1,test,0,1,37,127,10,10,14,0\n')
        self.local = self.odom(4.,6.,math.radians(60))
        self.glob = self.odom(7.,8.,math.radians(-20))
        self.gps = NavSatFix(); self.gps.status.status=0
        self.gps.latitude=37.;self.gps.longitude=127.

    def tearDown(self): self.tmp.cleanup()

    def test_current_rddf_projects_onto_route_and_rejects_far_position(self):
        from rddf_initialization_core import RddfRouteMap
        routes = RddfRouteMap(self.root)
        result = v.current_rddf_match(routes, (4., 6., 0.), True)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['route'], 'test')
        self.assertAlmostEqual(result['distance'], math.sqrt(2))
        self.assertIn('1.41 m', result['text'])
        far = v.current_rddf_match(routes, (100., 100., 0.), True)
        self.assertFalse(far['accepted'])
        self.assertEqual(far['reason'], 'TOO_FAR')
        self.assertEqual(far['routes'], [])

    def test_current_rddf_clears_missing_or_invalid_position(self):
        from rddf_initialization_core import RddfRouteMap
        routes = RddfRouteMap(self.root)
        for position, valid, reason in (
                (None, True, 'NO_FRESH_GLOBAL'),
                ((5., 5., 0.), False, 'LOCALIZATION_INVALID'),
                ((math.nan, 0., 0.), True, 'INVALID_INPUT')):
            result = v.current_rddf_match(routes, position, valid)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['reason'], reason)
            self.assertEqual(result['routes'], [])

    def test_current_rddf_keeps_overlapping_names_ambiguous(self):
        from rddf_initialization_core import RddfRouteMap
        # Startup may accept equivalent geometry, but it cannot identify which
        # of two named RDDF pieces the vehicle is currently following.
        (self.root/'overlap.csv').write_text(
            (self.root/'route.csv').read_text().replace(',test,', ',overlap,'))
        routes = RddfRouteMap(self.root)
        self.assertTrue(routes.match(5., 5., 5.)['accepted'])
        result = v.current_rddf_match(routes, (5., 5., 0.), True)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'AMBIGUOUS_ROUTE')
        self.assertEqual(result['routes'], ['overlap', 'test'])

    @staticmethod
    def odom(x,y,yaw):
        msg=Odometry();msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        return msg

    def test_common_translation_preserves_yaw_and_separation(self):
        model=v.SceneData(self.root)
        for key,msg in [('local',self.local),('global',self.glob),('gps',self.gps)]:
            model.ingest(key,msg,100.,True)
        np.testing.assert_allclose(model.points('local')[0],[0,0,math.radians(60)])
        np.testing.assert_allclose(model.points('global')[0],[3,2,math.radians(-20)])
        self.assertEqual(self.local.pose.pose.position.x,4.)
        displayed = {key: model.points(key)[0] for key in ('local', 'global')}
        self.assertEqual(
            'Raw yaw 110.0° | Calibrated yaw 15.0° | Local yaw 60.0° | Global yaw -20.0°',
            v.heading_readout(displayed, math.radians(110), math.radians(15)),
        )
        del displayed['global']  # No vehicle at this time must not borrow IMU yaw.
        self.assertEqual(
            'Calibrated yaw 15.0° | Local yaw 60.0° | Global yaw 이 시점 표시 없음',
            v.heading_readout(displayed, calibrated_yaw=math.radians(15)),
        )

    def test_odometry_mode_displays_unmodified_poses_without_gps_or_rddf(self):
        model = v.SceneData(self.root, frame_mode='odometry')
        for key, message in [('local', self.local), ('global', self.glob)]:
            model.ingest(key, message, 100., True)
        self.assertFalse(model.routes)
        self.assertFalse(model.data['gps'])
        np.testing.assert_allclose(model.shift, [0., 0.])
        np.testing.assert_allclose(model.points('local')[0], [4., 6., math.radians(60.)])
        np.testing.assert_allclose(model.points('global')[0], [7., 8., math.radians(-20.)])
        self.assertEqual(model.summary['additional_viewer_yaw_rotation_rad'], 0.)
        self.assertEqual(model.summary['frame_mode'], 'odometry')
        displayed = {key: model.points(key)[0] for key in ('local', 'global')}
        self.assertEqual(v.heading_readout(displayed, calibrated_yaw=math.radians(15.)),
                         'Calibrated yaw 15.0° | Local yaw 60.0° | Global yaw -20.0°')

    def test_odometry_view_fits_observed_positions_with_minimum_stationary_extent(self):
        model = v.SceneData(self.root, frame_mode='odometry')
        center, span = model.view_bounds()
        np.testing.assert_allclose(center, [0., 0.])
        np.testing.assert_allclose(span, [20., 20.])
        model.ingest('local', self.local, 100., True)
        model.ingest('global', self.glob, 100., True)
        center, span = model.view_bounds()
        np.testing.assert_allclose(center, [5.5, 7.])
        np.testing.assert_allclose(span, [20., 20.])
        model.ingest('global', self.odom(47., 8., 0.), 101., True)
        center, span = model.view_bounds()
        np.testing.assert_allclose(center, [25.5, 7.])
        np.testing.assert_allclose(span, [43., 20.])

    def test_odometry_rewind_keeps_zero_translation_without_waiting_for_gps(self):
        model = v.SceneData(self.root, frame_mode='odometry')
        model.ingest('local', self.local, 100., True)
        model.ingest('local', self.glob, 90., True)
        self.assertEqual(model.summary['clock_resets'], 1)
        np.testing.assert_allclose(model.shift, [0., 0.])
        np.testing.assert_allclose(model.points('local')[0], [7., 8., math.radians(-20.)])

    def test_live_recorded_same_input_same_result(self):
        live=v.SceneData(self.root)
        bagpath=self.root/'computed.bag'
        with rosbag.Bag(str(bagpath),'w') as bag:
            for i,(key,msg) in enumerate([('local',self.local),('gps',self.gps),('global',self.glob)]):
                stamp=100+i*.1
                live.ingest(key,msg,stamp,True)
                bag.write(v.TOPICS[key],msg,rospy.Time.from_sec(stamp))
        recorded=v.load_data(bagpath,None,self.root,200000)
        for key in ('local','global','gps'):
            np.testing.assert_allclose(live.points(key),recorded.points(key))
            np.testing.assert_allclose(live.times[key],recorded.times[key])

    def test_clock_rewind_clears_old_anchor(self):
        model=v.SceneData(self.root)
        model.ingest('local',self.local,100,True);model.ingest('gps',self.gps,101,True)
        model.ingest('local',self.glob,90,True)
        self.assertIsNone(model.shift);self.assertEqual(model.times['local'],[0.])
        self.assertFalse(model.data['gps']);self.assertEqual(model.summary['clock_resets'],1)

    def test_invalid_gps_cannot_anchor(self):
        model=v.SceneData(self.root);model.ingest('local',self.local,100,True)
        self.gps.latitude=91;model.ingest('gps',self.gps,100,True)
        self.assertIsNone(model.shift)

    def test_bounded_buffer_and_earlier_selection(self):
        model=v.SceneData(self.root,limit=3)
        for i in range(6): model.ingest('local',self.local,100+i,True)
        self.assertEqual(len(model.data['local']),3)
        self.assertEqual(model.summary['buffer_dropped'],3)
        self.assertEqual(v.bisect.bisect_right(model.times['local'],4.),2)

    def test_status_panel_uses_only_localization_diagnostics(self):
        message = DiagnosticArray(status=[
            DiagnosticStatus(level=DiagnosticStatus.OK, name='IMU',
                             message='OK', hardware_id='mando_localization',
                             values=[KeyValue(key='received', value='true')]),
            DiagnosticStatus(level=DiagnosticStatus.ERROR, name='camera',
                             message='ERROR', hardware_id='other_system'),
        ])
        decoded = v.decode_sample('diagnostics', message, {})
        self.assertEqual({'IMU'}, set(decoded))
        self.assertEqual(DiagnosticStatus.OK, decoded['IMU']['level'])
        self.assertEqual('true', decoded['IMU']['values']['received'])

    def test_raw_imu_reception_is_independent_of_calibrated_diagnostic(self):
        self.assertEqual(
            (0, '연결됨 · 데이터 수신 중'),
            v.stream_reception_status([1.0, 1.1], 1.5, 0.5),
        )
        self.assertEqual(
            (2, '수신 끊김'),
            v.stream_reception_status([1.0, 1.1], 1.500001, 0.4),
        )
        self.assertEqual(
            (1, '기록 없음'),
            v.stream_reception_status([2.0], 1.0, 0.5, '기록 없음'),
        )
        with self.assertRaises(ValueError):
            v.stream_reception_status([], 1.0, 0.0)

if __name__ == '__main__': unittest.main()
