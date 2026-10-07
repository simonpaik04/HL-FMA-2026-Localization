#!/usr/bin/python3
"""ROS checks for manual startup approval and independent output fail-closed behavior."""
import math
import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


class StartupGateTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.state = None
        self.valid = None
        self.status = None
        self.outputs = 0
        self.alive = 0
        self.send_ready = False
        self.ready_value = False
        self.send_valid = True
        self.pub = {
            'imu': rospy.Publisher('/molit/localization/imu/calibrated', Imu, queue_size=10),
            'encoder': rospy.Publisher('/erp42_serial/feedback', SerialFeedBack, queue_size=10),
            'twist': rospy.Publisher('/molit/vehicle/twist', TwistWithCovarianceStamped, queue_size=10),
            'local': rospy.Publisher('/molit/localization/local/odometry', Odometry, queue_size=10),
            'global': rospy.Publisher('/molit/localization/global/odometry', Odometry, queue_size=10),
            'ready': rospy.Publisher('/mando_localization/internal/initialization/ready', Bool, queue_size=10),
            'pose': rospy.Publisher('/mando_localization/internal/initialization/committed_pose',
                                    PoseWithCovarianceStamped, queue_size=10),
            'forced_valid': rospy.Publisher('/test/startup/forced_valid', Bool, queue_size=10),
        }
        self.subscribers = [
            rospy.Subscriber('/molit/localization/state', String,
                             lambda message: self.store('state', message.data)),
            rospy.Subscriber('/molit/localization/valid', Bool,
                             lambda message: self.store('valid', message.data)),
            rospy.Subscriber('/molit/localization/status', DiagnosticArray,
                             lambda message: self.store('status', message)),
            rospy.Subscriber('/test/startup/output', Odometry, self.output),
        ]
        self.timer = rospy.Timer(rospy.Duration(0.025), self.publish_motion)
        self.wait_for(lambda: all(p.get_num_connections() > 0 for p in self.pub.values()))

    def tearDown(self):
        self.timer.shutdown()

    def store(self, key, value):
        with self.lock:
            setattr(self, key, value)

    def output(self, _message):
        with self.lock:
            self.outputs += 1

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.025)
        self.assertTrue(predicate(), 'condition timed out; state=%s valid=%s' % (self.state, self.valid))

    def publish_motion(self, _event):
        now = rospy.Time.now()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = now, 'imu_link'
        imu.orientation.w = 1.0
        self.pub['imu'].publish(imu)
        feedback = SerialFeedBack()
        feedback.alive = self.alive
        self.alive = (self.alive + 1) % 256
        self.pub['encoder'].publish(feedback)
        twist = TwistWithCovarianceStamped()
        twist.header.stamp, twist.header.frame_id = now, 'base_link'
        self.pub['twist'].publish(twist)
        for kind, frame in (('local', 'odom'), ('global', 'map')):
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = now, frame
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = 30.0
            odom.pose.pose.orientation.w = 1.0
            odom.pose.covariance[0] = odom.pose.covariance[7] = 0.25
            self.pub[kind].publish(odom)
        if self.send_ready:
            self.pub['ready'].publish(Bool(self.ready_value))
        if self.send_valid:
            self.pub['forced_valid'].publish(Bool(True))

    @staticmethod
    def pose(frame='map', age=0.0):
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now() - rospy.Duration(age)
        message.header.frame_id = frame
        message.pose.pose.position.x = 30.0
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = math.radians(10.0)**2
        return message

    def test_manual_anchor_and_ready_watchdog(self):
        # Healthy motion and even a forced valid=true cannot bypass missing startup.
        self.wait_for(lambda: self.state == 'INITIALIZING' and self.valid is False)
        time.sleep(0.15)
        self.assertEqual(self.outputs, 0)
        self.send_ready, self.ready_value = True, True
        time.sleep(0.15)
        self.assertEqual(self.state, 'INITIALIZING')
        self.assertFalse(self.valid)
        for invalid in (self.pose('odom'), self.pose(age=2.0)):
            self.pub['pose'].publish(invalid)
            time.sleep(0.10)
            self.assertFalse(self.valid)
        invalid = self.pose()
        invalid.pose.covariance[35] = -1.0
        self.pub['pose'].publish(invalid)
        time.sleep(0.10)
        self.assertFalse(self.valid)

        # Ready first, committed pose second: starts bounded DR without GPS health.
        self.pub['pose'].publish(self.pose())
        self.wait_for(lambda: self.state == 'DEAD_RECKONING' and self.valid is True)
        self.wait_for(lambda: self.status is not None and any(s.name == 'RDDF_INITIALIZATION'
                                                              for s in self.status.status))
        gps = next(s for s in self.status.status if s.name == 'GPS_POSE')
        self.assertEqual(gps.message, 'NOT_RECEIVED')
        self.wait_for(lambda: self.outputs > 0)
        # Repeated commits and a live ready heartbeat must not renew the 2 s budget.
        deadline = time.monotonic() + 2.2
        while time.monotonic() < deadline:
            self.pub['pose'].publish(self.pose())
            time.sleep(0.10)
        self.wait_for(lambda: self.state == 'FAULT' and self.valid is False)

        # Loss of initializer heartbeat blocks output even with forced valid=true.
        self.send_ready = False
        time.sleep(0.70)
        count = self.outputs
        time.sleep(0.15)
        self.assertEqual(count, self.outputs)
        self.assertEqual(self.state, 'INITIALIZING')
        self.assertFalse(self.valid)

        # A new initialization requires another commit; pose-first ordering works.
        self.send_ready, self.ready_value = True, False
        time.sleep(0.10)
        self.pub['pose'].publish(self.pose())
        time.sleep(0.10)
        self.assertFalse(self.valid)
        self.ready_value = True
        self.wait_for(lambda: self.state == 'DEAD_RECKONING' and self.valid is True)
        self.wait_for(lambda: self.outputs > count)


if __name__ == '__main__':
    rospy.init_node('test_startup_gate')
    rostest.rosrun('mando_localization', 'startup_gate', StartupGateTest)
