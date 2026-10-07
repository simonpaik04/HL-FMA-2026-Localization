#!/usr/bin/python3
"""실제 Local/GPS gate/Coordinator/Global 노드에 회전 중 지연 GPS를 넣는다.

물리 장치 시험이 아닌 /clock 기반 합성 궤적이다. 측정 stamp를 보존해
0.3초 전달 지연을 주고 GPS 레버암 보정 및 최종 위치의 시각을 검증한다.
"""
import heapq
import math
import threading
import time
import unittest

import rospy
import rostest
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix


EPOCH = 1800000000.0


def trajectory(t):
    elapsed = max(0.0, t - 3.0)
    yaw = 0.30 * elapsed
    return (5.0 * math.sin(yaw), 5.0 * (1.0 - math.cos(yaw)), yaw,
            1.5 if t >= 3.0 else 0.0, 0.30 if t >= 3.0 else 0.0)


class DelayedGpsEkfTest(unittest.TestCase):
    def test_turning_gps_is_corrected_at_measurement_epoch(self):
        lock = threading.Lock()
        candidates, final_poses, local_poses = [], [], []

        def append(target, message):
            with lock:
                target.append(message)

        subscribers = [
            rospy.Subscriber('/mando_localization/internal/gps/candidate_pose',
                             PoseWithCovarianceStamped,
                             lambda m: append(candidates, m), queue_size=200),
            rospy.Subscriber('/molit/localization/odometry', Odometry,
                             lambda m: append(final_poses, m), queue_size=200),
            rospy.Subscriber('/molit/localization/local/odometry', Odometry,
                             lambda m: append(local_poses, m), queue_size=200),
        ]
        clock = rospy.Publisher('/clock', Clock, queue_size=10, latch=True)
        imu_pub = rospy.Publisher('/molit/sensors/imu/data', Imu, queue_size=100)
        encoder_pub = rospy.Publisher('/erp42_serial/feedback', SerialFeedBack,
                                      queue_size=30)
        gps_pub = rospy.Publisher('/molit/sensors/gps/fix', NavSatFix, queue_size=30)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            clock.publish(Clock(rospy.Time.from_sec(EPOCH)))
            if all(p.get_num_connections() for p in (imu_pub, encoder_pub, gps_pub)):
                break
            time.sleep(0.02)
        self.assertTrue(all(p.get_num_connections()
                            for p in (imu_pub, encoder_pub, gps_pub)))

        lat0 = math.radians(37.0)
        e2 = 6.69437999014e-3
        denom = math.sqrt(1 - e2 * math.sin(lat0)**2)
        east_scale = 6378137.0 / denom * math.cos(lat0)
        north_scale = 6378137.0 * (1 - e2) / denom**3
        pending = []
        alive = 0
        for tick in range(1801):
            t = tick / 100.0
            stamp = rospy.Time.from_sec(EPOCH + t)
            clock.publish(Clock(stamp))
            x, y, yaw, speed, omega = trajectory(t)
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
            imu.orientation.z = math.sin(yaw / 2)
            imu.orientation.w = math.cos(yaw / 2)
            imu.angular_velocity.z = omega
            imu.linear_acceleration.z = 9.80665
            for index in (0, 4, 8):
                imu.orientation_covariance[index] = 0.001
                imu.angular_velocity_covariance[index] = 0.001
                imu.linear_acceleration_covariance[index] = 0.01
            imu_pub.publish(imu)
            if tick % 5 == 0:
                feedback = SerialFeedBack()
                feedback.speed = speed
                feedback.alive = alive % 256
                alive += 1
                encoder_pub.publish(feedback)
            if tick % 10 == 0:
                fix = NavSatFix()
                fix.header.stamp, fix.header.frame_id = stamp, 'gps_link'
                fix.status.status, fix.status.service = 0, 1
                # first_fix 원점은 첫 안테나 위치이며 차체와의 레버암을 유지한다.
                east = x + 0.65 * math.cos(yaw) - 0.65
                north = y + 0.65 * math.sin(yaw)
                fix.latitude = 37.0 + math.degrees(north / north_scale)
                fix.longitude = 127.0 + math.degrees(east / east_scale)
                fix.altitude = 10.0
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                fix.position_covariance[0] = fix.position_covariance[4] = 0.04
                fix.position_covariance[8] = 1.0
                delay = 0.0 if t < 4.0 else 0.30
                heapq.heappush(pending, (t + delay, tick, fix))
            while pending and pending[0][0] <= t + 1e-8:
                gps_pub.publish(heapq.heappop(pending)[2])
            time.sleep(0.01)

        time.sleep(0.10)
        with lock:
            measured = [m for m in candidates
                        if 5.0 <= m.header.stamp.to_sec() - EPOCH <= 17.0]
            outputs = [m for m in final_poses
                       if 6.0 <= m.header.stamp.to_sec() - EPOCH <= 17.0]
        self.assertGreater(len(measured), 90, 'Delayed GPS rejected before EKF')
        self.assertGreater(len(outputs), 150, 'No fresh authorized final Odometry')
        candidate_errors = []
        output_errors = []
        for m in measured:
            t = m.header.stamp.to_sec() - EPOCH
            self.assertAlmostEqual(t * 10, round(t * 10), delta=5e-5,
                                   msg='GPS measurement timestamp was replaced')
            x, y = trajectory(t)[:2]
            p = m.pose.pose.position
            candidate_errors.append(math.hypot(p.x - x, p.y - y))
        for m in outputs:
            x, y = trajectory(m.header.stamp.to_sec() - EPOCH)[:2]
            p = m.pose.pose.position
            output_errors.append(math.hypot(p.x - x, p.y - y))
        self.assertLess(max(candidate_errors), 0.08,
                        'Lever arm used arrival-time yaw instead of measurement yaw')
        output_errors.sort()
        self.assertLess(output_errors[int(0.95 * (len(output_errors) - 1))], 0.25,
                        'Delayed measurements were not reflected in current output')
        print('DELAYED_GPS_METRICS candidates={} outputs={} candidate_max_m={:.4f} '
              'output_p95_m={:.4f}'.format(len(measured), len(outputs),
                max(candidate_errors),
                output_errors[int(0.95 * (len(output_errors) - 1))]))
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('test_delayed_gps_ekf')
    rostest.rosrun('mando_localization', 'delayed_gps_ekf', DelayedGpsEkfTest)
