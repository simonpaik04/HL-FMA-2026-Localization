#!/usr/bin/python3
"""두 EKF에서 정지 중 80도 AHRS drift 차단과 재출발 회전 반영을 검증한다."""
import math
import threading
import time
import unittest

import rospy
import rostest
from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

EPOCH = 1800000000.


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def delta(a, b):
    return math.degrees(math.atan2(math.sin(a-b), math.cos(a-b)))


class EncoderYawHoldRosTest(unittest.TestCase):
    def test_stop_drift_and_restart(self):
        lock = threading.Lock()
        records = {name: [] for name in ('normalized', 'calibrated', 'local', 'global')}

        def receive(name, message):
            with lock:
                records[name].append(message)

        subscribers = [rospy.Subscriber(topic, kind, lambda m, n=name: receive(n, m), queue_size=1000)
                       for name, kind, topic in (
                           ('normalized', Imu, '/molit/localization/imu/normalized'),
                           ('calibrated', Imu, '/molit/localization/imu/calibrated'),
                           ('local', Odometry, '/molit/localization/local/odometry'),
                           ('global', Odometry, '/molit/localization/global/odometry'))]
        clock = rospy.Publisher('/clock', Clock, queue_size=10, latch=True)
        imu = rospy.Publisher('/molit/sensors/imu/data', Imu, queue_size=100)
        feedback = rospy.Publisher('/erp42_serial/feedback', SerialFeedBack, queue_size=20)
        deadline = time.monotonic()+8.
        while time.monotonic() < deadline:
            clock.publish(Clock(rospy.Time.from_sec(EPOCH)))
            if imu.get_num_connections() and feedback.get_num_connections() >= 2:
                break
            time.sleep(.02)
        self.assertGreaterEqual(feedback.get_num_connections(), 2)
        alive = 0
        for tick in range(1801):
            t = tick/100.
            clock.publish(Clock(rospy.Time.from_sec(EPOCH+t)))
            moving = 2. <= t < 7. or 12. <= t < 15.
            if tick % 10 == 0 and t < 17.:
                msg = SerialFeedBack()
                msg.alive, alive = alive, (alive+1) % 256
                msg.speed, msg.encoder = (1., 50) if moving else (0., 0)
                feedback.publish(msg)
            # 독립 생성: 실제 회전 30도 + 정지 drift 80도 + 재출발 회전 20도.
            angle = -70.+10.*max(0., min(t-2., 3.))+16.*max(0., min(t-7., 5.))+10.*max(0., min(t-12., 2.))
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = rospy.Time.from_sec(EPOCH+t), 'imu_link'
            msg.orientation.z, msg.orientation.w = math.sin(math.radians(angle)/2), math.cos(math.radians(angle)/2)
            msg.angular_velocity.z = math.radians(10. if 2. <= t < 5. or 12. <= t < 14. else 2.)
            msg.linear_acceleration.z = 9.81
            imu.publish(msg)
            time.sleep(.006)
        time.sleep(.2)
        with lock:
            def window(name, start, end):
                return [m for m in records[name] if start < m.header.stamp.to_sec()-EPOCH < end]

            held = window('calibrated', 8., 11.5)
            self.assertGreater(len(held), 250)
            self.assertLess(max(abs(delta(yaw(m.orientation), yaw(held[0].orientation))) for m in held), 1e-6)
            self.assertTrue(all(m.angular_velocity.z == 0. for m in held))
            raw = window('normalized', 8., 11.5)
            self.assertGreater(abs(delta(yaw(raw[-1].orientation), yaw(raw[0].orientation))), 50.)
            for name in ('local', 'global'):
                stopped = window(name, 8., 11.5)
                self.assertGreater(len(stopped), 60)
                span = max(abs(delta(yaw(m.pose.pose.orientation), yaw(stopped[0].pose.pose.orientation))) for m in stopped)
                self.assertLess(span, .15, name)
                restarted = window(name, 14.5, 15.)[-1]
                self.assertAlmostEqual(delta(yaw(restarted.pose.pose.orientation), yaw(stopped[-1].pose.pose.orientation)), 20., delta=1.)
                print('%s stopped yaw span %.6f deg; restart turn preserved' % (name, span))
            released = window('calibrated', 17.5, 18.)
            self.assertGreater(len(released), 30)
            self.assertTrue(all(m.angular_velocity.z != 0. for m in released))
            originals = {m.header.stamp.to_nsec(): m for m in records['normalized']}
            for m in records['calibrated']:
                original = originals[m.header.stamp.to_nsec()]
                self.assertEqual(m.header.frame_id, original.header.frame_id)
                self.assertEqual(m.linear_acceleration, original.linear_acceleration)
                self.assertEqual(m.linear_acceleration_covariance, original.linear_acceleration_covariance)
                self.assertEqual(m.angular_velocity_covariance, original.angular_velocity_covariance)


if __name__ == '__main__':
    rospy.init_node('test_encoder_yaw_hold')
    rostest.rosrun('mando_localization', 'encoder_yaw_hold', EncoderYawHoldRosTest)
