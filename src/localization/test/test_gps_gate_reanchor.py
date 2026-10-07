#!/usr/bin/env python3
"""GPS gate가 실제 anchor 적용 뒤에만 transaction ack를 내는지 확인한다."""

import copy
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
from mando_localization.msg import GpsGateReanchor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


GPS_FIX = "/molit/sensors/gps/fix"
LOCAL_ODOMETRY = "/molit/localization/local/odometry"
GPS_CANDIDATE = "/mando_localization/internal/gps/candidate_pose"
GPS_GATE_POSE = "/mando_localization/internal/gps/gate_pose"
GPS_REANCHOR = "/mando_localization/internal/gps/reanchor_pose"
GPS_REANCHOR_ACCEPTED = "/mando_localization/internal/gps/reanchor_accepted"


class GpsGateReanchorTest(unittest.TestCase):
    def setUp(self):
        self.candidates = []
        self.gate_poses = []
        self.acks = []
        self.candidate_event = threading.Event()
        self.gate_pose_event = threading.Event()
        self.ack_event = threading.Event()

        self.candidate_subscriber = rospy.Subscriber(
            GPS_CANDIDATE, PoseWithCovarianceStamped, self._candidate_callback
        )
        self.gate_pose_subscriber = rospy.Subscriber(
            GPS_GATE_POSE, PoseWithCovarianceStamped, self._gate_pose_callback
        )
        self.ack_subscriber = rospy.Subscriber(
            GPS_REANCHOR_ACCEPTED, GpsGateReanchor, self._ack_callback
        )
        self.fix_publisher = rospy.Publisher(GPS_FIX, NavSatFix, queue_size=10)
        self.local_publisher = rospy.Publisher(
            LOCAL_ODOMETRY, Odometry, queue_size=10
        )
        self.reanchor_publisher = rospy.Publisher(
            GPS_REANCHOR, GpsGateReanchor, queue_size=10
        )

    def _candidate_callback(self, message):
        self.candidates.append(message)
        self.candidate_event.set()

    def _gate_pose_callback(self, message):
        self.gate_poses.append(message)
        self.gate_pose_event.set()

    def _ack_callback(self, message):
        self.acks.append(message)
        self.ack_event.set()

    def _wait_for_connections(self, timeout_sec=5.0):
        publishers = [
            self.fix_publisher,
            self.local_publisher,
            self.reanchor_publisher,
        ]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if all(publisher.get_num_connections() > 0 for publisher in publishers):
                return
            rospy.sleep(0.02)
        self.fail("GPS gate 테스트 publisher 연결이 완료되지 않았습니다.")

    def _publish_local(self, x, yaw_rad=0.0):
        self.current_local_x = x
        self.current_local_yaw = yaw_rad
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.z = math.sin(0.5 * yaw_rad)
        message.pose.pose.orientation.w = math.cos(0.5 * yaw_rad)
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 0.25
        self.local_publisher.publish(message)
        rospy.sleep(0.02)

    def _publish_fix(self, latitude=37.0, longitude=127.0):
        message = NavSatFix()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "gps_link"
        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = latitude
        message.longitude = longitude
        message.altitude = 10.0
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        message.position_covariance[0] = 0.25
        message.position_covariance[4] = 0.25
        message.position_covariance[8] = 1.0
        self.fix_publisher.publish(message)
        # GPS callback may precede its right-hand Local interpolation bracket.
        rospy.sleep(0.005)
        self._publish_local(self.current_local_x, self.current_local_yaw)
        rospy.sleep(0.035)

    def test_reanchor_ack_means_anchor_was_applied(self):
        self._wait_for_connections()

        # 최초 3회 정상 fix로 gate의 map=0/local=0 anchor를 만든다.
        for _ in range(3):
            self._publish_local(0.0)
            self._publish_fix()
        self.assertTrue(self.gate_pose_event.wait(2.0))
        self.assertAlmostEqual(0.0, self.gate_poses[-1].pose.pose.position.x)

        # 차량 base_link는 정지한 채 90도 회전하면 0.65 m 전방 안테나는
        # ENU에서 (+0.65, 0) -> (0, +0.65)로 움직인다. 레버암 보정 뒤의
        # GPS 후보는 base_link 원점에 그대로 있어야 한다.
        latitude_rad = math.radians(37.0)
        eccentricity_squared = 6.69437999014e-3
        denominator = math.sqrt(
            1.0 - eccentricity_squared * math.sin(latitude_rad) ** 2
        )
        prime_vertical_radius_m = 6378137.0 / denominator
        meridian_radius_m = (
            6378137.0
            * (1.0 - eccentricity_squared)
            / denominator**3
        )
        rotated_latitude = 37.0 + math.degrees(0.65 / meridian_radius_m)
        rotated_longitude = 127.0 + math.degrees(
            -0.65 / (prime_vertical_radius_m * math.cos(latitude_rad))
        )

        self.candidate_event.clear()
        self.gate_pose_event.clear()
        self._publish_local(0.0, math.pi / 2.0)
        self._publish_fix(rotated_latitude, rotated_longitude)
        self.assertTrue(self.candidate_event.wait(1.0))
        corrected = self.candidates[-1].pose.pose.position
        self.assertAlmostEqual(0.0, corrected.x, delta=0.01)
        self.assertAlmostEqual(0.0, corrected.y, delta=0.01)

        # 아래 reanchor 검증은 기존 datum 자세와 GPS 위치에서 계속한다.
        self._publish_local(0.0)
        self._publish_fix()

        # Local이 20 m drift한 상태에서는 같은 GPS를 기존 innovation gate가 거부한다.
        self.gate_pose_event.clear()
        self.candidate_event.clear()
        self._publish_local(20.0)
        self._publish_fix()
        self.assertTrue(self.candidate_event.wait(1.0))
        self.assertFalse(self.gate_pose_event.wait(0.15))
        candidate = self.candidates[-1]

        # 최신 GPS 후보에서 2 m 넘게 벗어난 command는 적용/ack되지 않는다.
        bad_command = GpsGateReanchor()
        bad_command.transaction_id = 41
        bad_command.pose = copy.deepcopy(candidate)
        bad_command.pose.pose.pose.position.x += 3.0
        self._publish_local(20.0)
        self.reanchor_publisher.publish(bad_command)
        self.assertFalse(self.ack_event.wait(0.12))

        # 정확한 transaction pose는 적용 후 그대로 ack된다.
        command = GpsGateReanchor()
        command.transaction_id = 42
        command.pose = copy.deepcopy(candidate)
        # The command refers to the candidate at local x=20, while the newest
        # Local has already advanced to x=25 during the transaction round trip.
        self._publish_local(25.0)
        self.reanchor_publisher.publish(command)
        self.assertTrue(self.ack_event.wait(1.0))
        self.assertEqual(42, self.acks[-1].transaction_id)
        self.assertEqual(command.pose.header.stamp, self.acks[-1].pose.header.stamp)

        # Anchor must remain map=0/local=20 at candidate time, not newest x=25.
        self.gate_pose_event.clear()
        self._publish_local(25.0)
        moved_longitude = 127.0 + math.degrees(
            5.0 / (prime_vertical_radius_m * math.cos(latitude_rad))
        )
        self._publish_fix(longitude=moved_longitude)
        self.assertTrue(self.gate_pose_event.wait(1.0))
        self.assertAlmostEqual(5.0, self.gate_poses[-1].pose.pose.position.x, delta=0.01)


if __name__ == "__main__":
    rospy.init_node("test_gps_gate_reanchor")
    rostest.rosrun(
        "mando_localization", "test_gps_gate_reanchor", GpsGateReanchorTest
    )
