#!/usr/bin/python3
"""엔코더 선도착·늦은 장착 TF·초기 yaw·실제 선회·후속 GNSS 정합을 두 EKF로 확인한다."""
from datetime import datetime
import math
import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from ublox_msgs.msg import NavPVT

EPOCH = 1800000000.


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class InitialHeadingRosTest(unittest.TestCase):
    def test_startup_turn_and_gnss_refinement(self):
        target = rospy.get_param('/calibrated_imu/initial_heading/yaw_rad')
        mount_yaw = math.radians(20.)
        lock = threading.Lock()
        normalized, calibrated = {}, {}
        local, global_, statuses = [], [], []

        def store(target_, message):
            with lock:
                if isinstance(target_, dict):
                    target_[message.header.stamp.to_nsec()] = message
                else:
                    target_.append(message)

        subscribers = [rospy.Subscriber(topic, kind, lambda m, dst=dst: store(dst, m), queue_size=500)
                       for topic, kind, dst in (
                           ('/molit/localization/imu/normalized', Imu, normalized),
                           ('/molit/localization/imu/calibrated', Imu, calibrated),
                           ('/molit/localization/local/odometry', Odometry, local),
                           ('/molit/localization/global/odometry', Odometry, global_),
                           ('/mando_localization/internal/imu/calibration_status', DiagnosticArray, statuses))]
        clock = rospy.Publisher('/clock', Clock, queue_size=10, latch=True)
        imu = rospy.Publisher('/molit/sensors/imu/data', Imu, queue_size=100)
        feedback = rospy.Publisher('/erp42_serial/feedback', SerialFeedBack, queue_size=20)
        gnss = rospy.Publisher('/molit/sensors/gps/navpvt', NavPVT, queue_size=20)
        ready = rospy.Publisher('/mando_localization/internal/timing/clock_ready', Bool, queue_size=10)
        tf = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        deadline = time.monotonic()+8.
        while time.monotonic() < deadline:
            clock.publish(Clock(rospy.Time.from_sec(EPOCH)))
            if all(pub.get_num_connections() for pub in (imu, feedback, gnss, ready, tf)):
                break
            time.sleep(.02)
        self.assertTrue(all(pub.get_num_connections() for pub in (imu, feedback, gnss, ready, tf)))
        alive = 0
        for tick in range(1801):
            t = tick/100.
            clock.publish(Clock(rospy.Time.from_sec(EPOCH+t)))
            if tick % 10 == 0:
                ready.publish(Bool(t < 15.))
                msg = SerialFeedBack()
                msg.alive, alive = alive, (alive+1) % 256
                msg.speed = 2. if 4. <= t < 15. else 0.
                msg.encoder = 100 if msg.speed else 0
                feedback.publish(msg)
            if tick == 120:
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'base_link', 'imu_link'
                transform.transform.rotation.z = math.sin(mount_yaw/2)
                transform.transform.rotation.w = math.cos(mount_yaw/2)
                tf.publish(TFMessage([transform]))
            if t >= .5:
                turn = math.radians(10.)*max(0., min(t-4., 3.))
                angle = math.radians(-70.)+mount_yaw+turn
                msg = Imu()
                msg.header.stamp, msg.header.frame_id = rospy.Time.from_sec(EPOCH+t), 'imu_link'
                msg.orientation.z, msg.orientation.w = math.sin(angle/2), math.cos(angle/2)
                msg.angular_velocity.z = math.radians(10.) if 4. <= t < 7. else 0.
                msg.linear_acceleration.z = 9.81
                imu.publish(msg)
            if tick % 10 == 0 and 8. <= t < 15.:
                measured = EPOCH+t-.25
                date = datetime.utcfromtimestamp(measured)
                course = target+math.radians(35.)
                msg = NavPVT()
                msg.year, msg.month, msg.day = date.year, date.month, date.day
                msg.hour, msg.min, msg.sec = date.hour, date.minute, date.second
                msg.nano = int(round((measured-int(measured))*1e9))
                msg.valid, msg.flags2, msg.fixType, msg.flags, msg.numSV = 7, 32, 3, 1, 12
                msg.heading = int(round(math.degrees(math.pi/2-course)*1e5))
                msg.headAcc, msg.hAcc, msg.gSpeed, msg.sAcc = 200000, 300, 2000, 100
                msg.velE, msg.velN = int(round(2000*math.cos(course))), int(round(2000*math.sin(course)))
                distance = 2.*(t-.25-8.)
                msg.lat = int(round((37.+math.degrees(distance*math.sin(course)/6378137.))*1e7))
                msg.lon = int(round((127.+math.degrees(distance*math.cos(course)/(6378137.*math.cos(math.radians(37.)))))*1e7))
                gnss.publish(msg)
            time.sleep(.006)
        time.sleep(.3)
        with lock:
            keys = sorted(calibrated)
            self.assertGreater(len(keys), 1200)
            self.assertGreaterEqual(keys[0], int((EPOCH+1.2)*1e9))
            first = calibrated[keys[0]]
            self.assertAlmostEqual(error(yaw(first.orientation)-mount_yaw, target), 0., delta=1e-6)
            for key in set(keys) & set(normalized):
                a, b = normalized[key], calibrated[key]
                self.assertEqual(a.header.stamp, b.header.stamp)
                self.assertEqual(a.header.frame_id, b.header.frame_id)
                self.assertEqual(a.angular_velocity, b.angular_velocity)
                self.assertEqual(a.linear_acceleration, b.linear_acceleration)
                self.assertEqual(a.angular_velocity_covariance, b.angular_velocity_covariance)
                self.assertEqual(a.linear_acceleration_covariance, b.linear_acceleration_covariance)
            for outputs in (local, global_):
                self.assertGreater(len(outputs), 300)
                early = [m for m in outputs if EPOCH <= m.header.stamp.to_sec() < EPOCH+.5]
                self.assertGreater(len(early), 3, 'encoder-first EKF output missing')
                self.assertTrue(all(abs(error(yaw(m.pose.pose.orientation), target)) < 1e-6 for m in early))
                turned = [m for m in outputs if EPOCH+7.5 < m.header.stamp.to_sec() < EPOCH+8.]
                self.assertGreater(len(turned), 3)
                self.assertAlmostEqual(error(yaw(turned[-1].pose.pose.orientation), target+math.radians(30.)), 0., delta=math.radians(3.))
                self.assertAlmostEqual(error(yaw(outputs[-1].pose.pose.orientation), target+math.radians(35.)), 0., delta=math.radians(3.))
            self.assertAlmostEqual(error(yaw(calibrated[keys[-1]].orientation)-mount_yaw,
                                         target+math.radians(35.)), 0., delta=math.radians(.1))
            messages = [s.message for a in statuses for s in a.status]
            self.assertTrue(any(s.startswith('RDDF_INITIALIZED') for s in messages))
            self.assertTrue(any(s.startswith('CALIBRATED') for s in messages))
            print('Initial body yaw %.6f deg; final Local/Global %.6f/%.6f deg; mount wait, raw fields, turn and GNSS refinement passed'
                  % (math.degrees(target), math.degrees(yaw(local[-1].pose.pose.orientation)),
                     math.degrees(yaw(global_[-1].pose.pose.orientation))))


if __name__ == '__main__':
    rospy.init_node('test_initial_heading')
    rostest.rosrun('mando_localization', 'initial_heading', InitialHeadingRosTest)
