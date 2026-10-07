#!/usr/bin/python3
"""실제 normalizer/CalibratedIMU/adapter/Local·Global EKF의 합성 지연 GNSS 시험."""
from datetime import datetime
import copy
import math
import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from ublox_msgs.msg import NavPVT


EPOCH = 1800000000.0


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def measurement(message):
    result = copy.deepcopy(message)
    # ROS publishers own Header.seq. Measurement stamp/frame and all payload remain comparable.
    result.header.seq = 0
    return result


class CalibratedImuRosTest(unittest.TestCase):
    def test_common_input_alignment_and_hold_reaches_both_ekfs(self):
        lock = threading.Lock()
        normalized, calibrated, internal = {}, {}, {}
        local, global_, statuses = [], [], []

        def imu_callback(target, message):
            with lock:
                target[message.header.stamp.to_nsec()] = message

        def append(target, message):
            with lock:
                target.append(message)

        subscribers = [
            rospy.Subscriber(topic, Imu, lambda m, target=target: imu_callback(target, m), queue_size=500)
            for topic, target in [('/molit/localization/imu/normalized', normalized),
                                  ('/molit/localization/imu/calibrated', calibrated),
                                  ('/mando_localization/internal/ekf/imu', internal)]
        ]
        subscribers.extend([
            rospy.Subscriber('/molit/localization/local/odometry', Odometry, lambda m: append(local, m), queue_size=500),
            rospy.Subscriber('/molit/localization/global/odometry', Odometry, lambda m: append(global_, m), queue_size=500),
            rospy.Subscriber('/mando_localization/internal/imu/calibration_status', DiagnosticArray,
                             lambda m: append(statuses, m), queue_size=100),
        ])
        clock = rospy.Publisher('/clock', Clock, queue_size=10, latch=True)
        imu_pub = rospy.Publisher('/molit/sensors/imu/data', Imu, queue_size=100)
        feedback_pub = rospy.Publisher('/erp42_serial/feedback', SerialFeedBack, queue_size=30)
        gps_pub = rospy.Publisher('/molit/sensors/gps/navpvt', NavPVT, queue_size=30)
        ready = rospy.Publisher('/mando_localization/internal/timing/clock_ready', Bool, queue_size=10)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            clock.publish(Clock(rospy.Time.from_sec(EPOCH)))
            if all(p.get_num_connections() for p in (imu_pub, feedback_pub, gps_pub, ready)):
                break
            time.sleep(0.02)
        self.assertTrue(all(p.get_num_connections() for p in (imu_pub, feedback_pub, gps_pub, ready)))
        gnss_offset = rospy.get_param("~gnss_time_offset_sec", -0.25)
        publish_sleep = rospy.get_param("~publish_sleep_sec", 0.004)
        alive = 0
        for tick in range(3601):
            t = tick/100.
            clock.publish(Clock(rospy.Time.from_sec(EPOCH+t)))
            if tick % 10 == 0:
                # GNSS disappears and clock readiness becomes false after alignment.
                ready.publish(Bool(t < 14.))
                feedback = SerialFeedBack()
                feedback.alive = alive
                alive = (alive+1) % 256
                feedback.speed = 2.0 if 2.0 <= t < 14.0 else 0.0
                feedback.encoder = 100 if feedback.speed else 0
                feedback_pub.publish(feedback)
            message = Imu()
            message.header.seq = tick
            message.header.stamp = rospy.Time.from_sec(EPOCH+t)
            message.header.frame_id = 'imu_link'
            message.orientation.z = math.sin(math.radians(-70)/2)
            message.orientation.w = math.cos(math.radians(-70)/2)
            message.linear_acceleration.z = 9.81
            imu_pub.publish(message)
            if tick % 10 == 0 and 2.5 <= t < 14.:
                measured = EPOCH+t+gnss_offset
                seconds = int(measured)
                date = datetime.utcfromtimestamp(seconds)
                gps = NavPVT()
                gps.year, gps.month, gps.day = date.year, date.month, date.day
                gps.hour, gps.min, gps.sec = date.hour, date.minute, date.second
                gps.nano = int(round((measured-seconds)*1e9))
                gps.valid, gps.flags2 = 7, 32
                gps.fixType, gps.flags, gps.numSV = 3, 1, 12
                gps.heading, gps.headAcc = 9000000, 200000  # East, 2 deg estimate
                gps.velE, gps.gSpeed, gps.sAcc = 2000, 2000, 100
                gps.hAcc = 300
                gps.lat = int(37.*1e7)
                east = (t+gnss_offset-2.)*2.
                gps.lon = int(round((127.+math.degrees(east/(6378137.*math.cos(math.radians(37.)))))*1e7))
                gps_pub.publish(gps)
            time.sleep(publish_sleep)
        time.sleep(0.4)
        with lock:
            common = sorted(set(normalized) & set(calibrated) & set(internal))
            self.assertGreater(len(common), 2500)
            first = [key for key in common if key < int((EPOCH+2.)*1e9)]
            self.assertGreater(len(first), 100)
            for key in first:
                self.assertEqual(measurement(normalized[key]), measurement(calibrated[key]))
            for key in common:
                a, b = normalized[key], calibrated[key]
                self.assertEqual(a.header.stamp, b.header.stamp)
                self.assertEqual(a.header.frame_id, b.header.frame_id)
                self.assertEqual(a.angular_velocity, b.angular_velocity)
                self.assertEqual(a.linear_acceleration, b.linear_acceleration)
                self.assertEqual(a.angular_velocity_covariance, b.angular_velocity_covariance)
                self.assertEqual(a.linear_acceleration_covariance, b.linear_acceleration_covariance)
                self.assertEqual(measurement(b), measurement(internal[key]))
            final = calibrated[common[-1]]
            self.assertAlmostEqual(0., math.degrees(yaw(final.orientation)), delta=0.1)
            self.assertGreater(final.orientation_covariance[8], math.radians(9.9)**2)
            for outputs in (local, global_):
                self.assertGreater(len(outputs), 400)
                self.assertAlmostEqual(0., math.degrees(yaw(outputs[-1].pose.pose.orientation)), delta=3.)
            self.assertTrue(any('ALIGNING' in s.message for array in statuses for s in array.status))
            self.assertTrue(any('CALIBRATED' in s.message for array in statuses for s in array.status))
            corrections = [int(value.value) for array in statuses for status in array.status
                           for value in status.values if value.key == 'correction_count']
            self.assertGreaterEqual(max(corrections), 2, 'production repeat mode must accept independent later windows')

            print('CalibratedIMU ROS proof: %d shared samples; final calibrated/local/global yaw=%.3f/%.3f/%.3f deg'
                  % (len(common), math.degrees(yaw(final.orientation)),
                     math.degrees(yaw(local[-1].pose.pose.orientation)),
                     math.degrees(yaw(global_[-1].pose.pose.orientation))))


if __name__ == '__main__':
    rospy.init_node('test_calibrated_imu')
    rostest.rosrun('mando_localization', 'calibrated_imu', CalibratedImuRosTest)
