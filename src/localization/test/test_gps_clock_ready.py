#!/usr/bin/env python3
"""Clock admission uses a fresh positive heartbeat, including while /clock pauses."""
import time
import unittest

import rospy
import rostest
from mando_localization.msg import GpsGateReanchor
from std_msgs.msg import Bool
from test_delayed_gps_alignment import DelayedGpsAlignmentTest


class GpsClockReadyTest(unittest.TestCase):
    setUp = DelayedGpsAlignmentTest.setUp
    set_clock = DelayedGpsAlignmentTest.set_clock
    publish_local = DelayedGpsAlignmentTest.publish_local
    publish_gps = DelayedGpsAlignmentTest.publish_gps
    wait_stamp = DelayedGpsAlignmentTest.wait_stamp

    def test_missing_false_and_stale_clock_block_fixes_and_reanchor(self):
        ready = rospy.Publisher('/mando_localization/internal/timing/clock_ready', Bool,
                                 queue_size=10, latch=True)
        reanchor = rospy.Publisher('/mando_localization/internal/gps/reanchor_pose',
                                    GpsGateReanchor, queue_size=10)
        acknowledgments = []
        ack_sub = rospy.Subscriber('/mando_localization/internal/gps/reanchor_accepted',
                                   GpsGateReanchor, acknowledgments.append)
        time.sleep(.1)

        # Missing heartbeat cannot admit even a fix with valid coordinates/time.
        self.publish_local(1000.)
        self.publish_gps(1000.)
        time.sleep(.08)
        self.assertFalse(self.outputs)
        ready.publish(Bool(False))
        time.sleep(.02)
        self.set_clock(1000.01)
        self.publish_local(1000.01)
        self.publish_gps(1000.01)
        time.sleep(.08)
        self.assertFalse(self.outputs)

        for value in (1000.02, 1000.03, 1000.04):
            ready.publish(Bool(True))
            self.set_clock(value)
            self.publish_local(value)
            self.publish_gps(value)
            time.sleep(.025)
        accepted = self.wait_stamp(1000.04)

        # Steady time advances while ROS time is paused: the heartbeat must expire.
        time.sleep(.35)
        self.set_clock(1000.05)
        self.publish_local(1000.05)
        self.publish_gps(1000.05)
        command = GpsGateReanchor()
        command.transaction_id = 42
        command.pose = accepted
        reanchor.publish(command)
        time.sleep(.08)
        self.assertFalse(any(m.header.stamp == rospy.Time.from_sec(1000.05) for m in self.outputs))
        self.assertFalse(acknowledgments)

        for value in (1000.06, 1000.07, 1000.08):
            ready.publish(Bool(True))
            self.set_clock(value)
            self.publish_local(value)
            self.publish_gps(value)
            time.sleep(.025)
        self.wait_stamp(1000.08)
        ack_sub.unregister()


if __name__ == '__main__':
    rospy.init_node('test_gps_clock_ready')
    rostest.rosrun('mando_localization', 'gps_clock_ready', GpsClockReadyTest)
