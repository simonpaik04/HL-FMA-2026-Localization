#!/usr/bin/env python3
"""Exercise the public publisher with real ROS messages and no viewer process."""
from pathlib import Path
import sys
import time
import unittest

import rospkg
import rospy
import rostest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mando_localization.msg import RddfMatch

PACKAGE = Path(rospkg.RosPack().get_path('mando_localization'))
sys.path.insert(0, str(PACKAGE/'scripts'))
from rddf_initialization_core import RddfRouteMap
from rddf_tracking_core import current_rddf_match


class PublicTopicTest(unittest.TestCase):
    def test_publisher_without_rviz(self):
        received = []
        published_stamps = set()
        subscriber = rospy.Subscriber('/molit/localization/rddf/current', RddfMatch,
                                       received.append, queue_size=10)
        odometry = rospy.Publisher('/molit/localization/global/odometry', Odometry, queue_size=1)
        validity = rospy.Publisher('/molit/localization/valid', Bool, queue_size=1)
        routes = RddfRouteMap(PACKAGE/'rddf')
        position = expected = None
        for points in routes.routes.values():
            for point in points[::20]:
                result = current_rddf_match(routes, point, True)
                if result['accepted']:
                    position, expected = point, result['route']
                    break
            if position is not None:
                break
        self.assertIsNotNone(position, 'test map needs a uniquely identifiable point')

        def await_reason(reason, publish=None, route=None, source=None):
            started = rospy.Time.now()
            deadline = time.monotonic()+7
            while time.monotonic() < deadline:
                if publish:
                    publish()
                time.sleep(.05)
                for message in reversed(received):
                    reason_matches = (message.matched if reason == 'MATCHED'
                                      else message.reason == reason)
                    if (message.header.stamp >= started and reason_matches
                            and (route is None or message.route_name == route)
                            and (source is None or message.source_route_name == source)):
                        return message
            self.fail('missing {} (last: {})'.format(reason, received[-1] if received else None))

        def publish(valid=True, frame='map', age=0., point=position):
            message = Odometry()
            message.header.stamp = rospy.Time.now()-rospy.Duration(age)
            published_stamps.add(message.header.stamp.to_nsec())
            message.header.frame_id = frame
            message.pose.pose.position.x, message.pose.pose.position.y = map(float, point)
            message.pose.pose.orientation.w = 1.
            odometry.publish(message)
            validity.publish(Bool(valid))

        self.assertFalse(await_reason('NO_GLOBAL').matched)
        matched = await_reason('MATCHED', publish)
        self.assertEqual(matched.route_name, expected)
        self.assertTrue(matched.has_nearest)
        self.assertEqual(matched.header.frame_id, 'map')
        self.assertGreater(matched.pose_stamp.to_sec(), 0.)
        self.assertIn(matched.pose_stamp.to_nsec(), published_stamps)
        for reason, publisher in [
                ('LOCALIZATION_INVALID', lambda: publish(valid=False)),
                ('FRAME_MISMATCH', lambda: publish(frame='odom')),
                ('STALE_GLOBAL', lambda: publish(age=2.)),
                ('TOO_FAR', lambda: publish(point=(1e6, 1e6)))]:
            message = await_reason(reason, publisher)
            self.assertFalse(message.matched)
            self.assertEqual(message.route_name, '')
            self.assertEqual(message.segment_index, -1)
            self.assertEqual(message.has_nearest, reason == 'TOO_FAR')
        await_reason('MATCHED', publish)
        self.assertFalse(await_reason('STALE_GLOBAL').has_nearest)
        # Cross-route teleporting belongs to startup/unit tests. Once this
        # stateful public node acquires a route it deliberately preserves that
        # route through crossings instead of globally reacquiring every tick.
        subscriber.unregister()
        odometry.unregister()
        validity.unregister()


if __name__ == '__main__':
    rospy.init_node('test_rddf_tracking_public_topic')
    rostest.rosrun('mando_localization', 'rddf_tracking_public_topic', PublicTopicTest)
