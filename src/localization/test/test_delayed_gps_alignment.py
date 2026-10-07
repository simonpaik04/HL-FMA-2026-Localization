#!/usr/bin/env python3
"""Delayed turning GPS must use historical Local yaw and preserve measurement time."""
import copy
import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix, NavSatStatus


class DelayedGpsAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.outputs = []
        self.clock = rospy.Publisher('/clock', Clock, queue_size=10, latch=True)
        self.local = rospy.Publisher('/molit/localization/local/odometry', Odometry, queue_size=50)
        self.gps = rospy.Publisher('/molit/sensors/gps/fix', NavSatFix, queue_size=20)
        self.sub = rospy.Subscriber('/mando_localization/internal/gps/gate_pose',
                                    PoseWithCovarianceStamped, self.outputs.append)
        self.set_clock(1000.)
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            if self.local.get_num_connections() and self.gps.get_num_connections():
                break
            time.sleep(.02)
        self.assertTrue(self.local.get_num_connections() and self.gps.get_num_connections())

    def set_clock(self, seconds):
        self.clock.publish(Clock(rospy.Time.from_sec(seconds)))
        time.sleep(.025)

    def publish_local(self, stamp, yaw=0., x=0.):
        message = Odometry()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = x
        message.pose.pose.orientation.z = math.sin(yaw/2.)
        message.pose.pose.orientation.w = math.cos(yaw/2.)
        for i in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[i] = .25
        self.local.publish(message)
        time.sleep(.025)

    def publish_gps(self, stamp, yaw=0., x=0.):
        # Antenna moves around a fixed rear-axle point: remove the datum lever arm.
        lat = math.radians(37.)
        e2 = 6.69437999014e-3
        denominator = math.sqrt(1-e2*math.sin(lat)**2)
        east_radius = 6378137./denominator * math.cos(lat)
        north_radius = 6378137.*(1-e2)/denominator**3
        message = NavSatFix()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = 'gps_link'
        message.status.status = NavSatStatus.STATUS_FIX
        message.latitude = 37. + math.degrees(.65*math.sin(yaw)/north_radius)
        message.longitude = 127. + math.degrees((x+.65*(math.cos(yaw)-1))/east_radius)
        message.altitude = 10.
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        message.position_covariance[0] = message.position_covariance[4] = .25
        message.position_covariance[8] = 1.
        self.gps.publish(message)
        return message

    def wait_stamp(self, seconds, timeout=1.):
        stamp = rospy.Time.from_sec(seconds)
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            matching = [m for m in self.outputs if m.header.stamp == stamp]
            if matching:
                return matching[-1]
            time.sleep(.01)
        self.fail('GPS measurement at %.3f was not accepted' % seconds)

    def test_delayed_turn_pending_duplicates_and_clock_rewind(self):
        for value in (1000., 1000.01, 1000.02):
            self.set_clock(value)
            self.publish_local(value)
            self.publish_gps(value)
            time.sleep(.03)
        self.wait_stamp(1000.02)

        # Build 0.4 s of Local history; GPS at +0.095 s arrives at +0.420 s.
        yaw_at = lambda value: (value-1000.02)*math.pi/.3
        for i in range(1, 9):
            value = 1000.02+i*.05
            self.set_clock(value)
            self.publish_local(value, yaw_at(value))
        delayed_stamp = 1000.095
        delayed = self.publish_gps(delayed_stamp, yaw_at(delayed_stamp))
        result = self.wait_stamp(delayed_stamp)
        self.assertAlmostEqual(0., result.pose.pose.position.x, delta=.005)
        self.assertAlmostEqual(0., result.pose.pose.position.y, delta=.005)

        # Duplicate/reordered packets cannot poison the anchor or restart recovery.
        count = len(self.outputs)
        duplicate = copy.deepcopy(delayed)
        duplicate.longitude += .001
        self.gps.publish(duplicate)
        duplicate.header.stamp = rospy.Time.from_sec(1000.08)
        self.gps.publish(duplicate)
        time.sleep(.05)
        self.assertEqual(count, len(self.outputs))
        next_stamp = 1000.145
        self.publish_gps(next_stamp, yaw_at(next_stamp))
        self.wait_stamp(next_stamp)

        # GPS arrives just before the right-hand Local bracket: no extrapolation.
        self.set_clock(1000.445)
        pending_stamp = 1000.445
        self.publish_gps(pending_stamp, yaw_at(pending_stamp))
        time.sleep(.02)
        self.assertFalse(any(m.header.stamp == rospy.Time.from_sec(pending_stamp) for m in self.outputs))
        self.set_clock(1000.47)
        self.publish_local(1000.47, yaw_at(1000.47))
        self.wait_stamp(pending_stamp)

        # A replay epoch rewind must forget the previous recovery count and datum.
        self.set_clock(500.)
        self.publish_local(500.)
        self.publish_gps(500.)
        time.sleep(.04)
        self.assertFalse(any(m.header.stamp == rospy.Time.from_sec(500.) for m in self.outputs))
        for value in (500.01, 500.02):
            self.set_clock(value)
            self.publish_local(value)
            self.publish_gps(value)
            time.sleep(.03)
        reset = self.wait_stamp(500.02)
        self.assertAlmostEqual(0., reset.pose.pose.position.x, delta=.005)


if __name__ == '__main__':
    rospy.init_node('test_delayed_gps_alignment')
    rostest.rosrun('mando_localization', 'delayed_gps_alignment', DelayedGpsAlignmentTest)
