#!/usr/bin/env python3
"""GPS-only reset이 정지·Global 3회 확인·gate ack 순서를 지키는지 검사한다."""

import copy
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from mando_localization.msg import GpsGateReanchor
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose, SetPoseResponse
from std_msgs.msg import String


GPS_GATE_POSE = "/mando_localization/internal/gps/gate_pose"
GPS_CANDIDATE = "/mando_localization/internal/gps/candidate_pose"
GPS_REANCHOR = "/mando_localization/internal/gps/reanchor_pose"
GPS_REANCHOR_ACCEPTED = "/mando_localization/internal/gps/reanchor_accepted"
TWIST = "/molit/vehicle/twist"
GLOBAL_ODOMETRY = "/molit/localization/global/odometry"
GPS_MAP_POSE = "/molit/localization/gps/map_pose"
RECOVERY_STATE = "/molit/localization/recovery/state"
SET_POSE = "/mando_localization/internal/ekf/global_set_pose"


class GpsOnlyRelocalizationTest(unittest.TestCase):
    def setUp(self):
        self.service_requests = []
        self.reanchors = []
        self.public_poses = []
        self.recovery_states = []
        self.service_event = threading.Event()
        self.reanchor_event = threading.Event()
        self.public_pose_event = threading.Event()

        self.set_pose_service = rospy.Service(
            SET_POSE, SetPose, self._set_pose_callback
        )
        self.reanchor_subscriber = rospy.Subscriber(
            GPS_REANCHOR, GpsGateReanchor, self._reanchor_callback
        )
        self.public_pose_subscriber = rospy.Subscriber(
            GPS_MAP_POSE, PoseWithCovarianceStamped, self._public_pose_callback
        )
        self.state_subscriber = rospy.Subscriber(
            RECOVERY_STATE, String, lambda message: self.recovery_states.append(message.data)
        )

        self.gate_pose_publisher = rospy.Publisher(
            GPS_GATE_POSE, PoseWithCovarianceStamped, queue_size=10
        )
        self.candidate_publisher = rospy.Publisher(
            GPS_CANDIDATE, PoseWithCovarianceStamped, queue_size=10
        )
        self.twist_publisher = rospy.Publisher(
            TWIST, TwistWithCovarianceStamped, queue_size=10
        )
        self.global_publisher = rospy.Publisher(
            GLOBAL_ODOMETRY, Odometry, queue_size=10
        )
        self.ack_publisher = rospy.Publisher(
            GPS_REANCHOR_ACCEPTED, GpsGateReanchor, queue_size=10
        )

    def _set_pose_callback(self, request):
        self.service_requests.append(copy.deepcopy(request))
        self.service_event.set()
        return SetPoseResponse()

    def _reanchor_callback(self, message):
        self.reanchors.append(message)
        self.reanchor_event.set()

    def _public_pose_callback(self, message):
        self.public_poses.append(message)
        self.public_pose_event.set()

    def _wait_for_connections(self, timeout_sec=5.0):
        publishers = [
            self.gate_pose_publisher,
            self.candidate_publisher,
            self.twist_publisher,
            self.global_publisher,
            self.ack_publisher,
        ]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if all(publisher.get_num_connections() > 0 for publisher in publishers):
                return
            rospy.sleep(0.02)
        self.fail("GPS-only 테스트 publisher 연결이 완료되지 않았습니다.")

    @staticmethod
    def _pose(x):
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 0.25
        return message

    def _publish_twist(self, speed=0.0):
        message = TwistWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.twist.twist.linear.x = speed
        self.twist_publisher.publish(message)

    def _publish_global(self, x, stamp=None):
        message = Odometry()
        message.header.stamp = stamp if stamp is not None else rospy.Time.now()
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.z = math.sin(0.25)
        message.pose.pose.orientation.w = math.cos(0.25)
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 0.25
        self.global_publisher.publish(message)
        rospy.sleep(0.04)

    def test_gps_only_reset_requires_three_new_global_outputs_and_gate_ack(self):
        self._wait_for_connections()

        self.gate_pose_publisher.publish(self._pose(0.0))
        self.assertTrue(self.public_pose_event.wait(1.0))
        initial_public_count = len(self.public_poses)
        self.public_pose_event.clear()
        rospy.sleep(0.6)

        # 이동 중 후보는 이후 정지 후보 카운트에 포함하지 않는다.
        self._publish_twist(1.0)
        self._publish_global(0.0)
        self.candidate_publisher.publish(self._pose(10.0))
        rospy.sleep(0.05)

        # 정지·fresh Global 조건에서 서로 다른 5개 GPS 후보를 모은다.
        for _ in range(4):
            self._publish_twist(0.0)
            self._publish_global(0.0)
            self.candidate_publisher.publish(self._pose(10.0))
            rospy.sleep(0.05)
        self.assertFalse(self.service_event.wait(0.1))

        self._publish_twist(0.0)
        self._publish_global(0.0)
        self.candidate_publisher.publish(self._pose(10.0))
        rospy.sleep(0.05)
        self.assertTrue(
            self.service_event.wait(1.0),
            "Global set_pose가 호출되지 않았습니다: {}".format(
                self.recovery_states[-12:]
            ),
        )
        request = self.service_requests[-1].pose
        self.assertAlmostEqual(10.0, request.pose.pose.position.x)
        self.assertAlmostEqual(math.sin(0.25), request.pose.pose.orientation.z)
        self.assertAlmostEqual(math.cos(0.25), request.pose.pose.orientation.w)

        reset_stamp = request.header.stamp
        # reset 이전 stamp는 세지 않고, 두 번 일치만으로도 reanchor하지 않는다.
        self._publish_global(10.0, reset_stamp - rospy.Duration.from_sec(0.01))
        self._publish_global(10.0)
        self._publish_global(10.0)
        self.assertFalse(self.reanchor_event.wait(0.1))

        # 중간 거리 불일치는 연속 확인을 0으로 되돌린다.
        self._publish_global(20.0)
        for _ in range(2):
            self._publish_global(10.0)
        self.assertFalse(self.reanchor_event.wait(0.1))
        self._publish_global(10.0)
        self.assertTrue(self.reanchor_event.wait(1.0))
        self.assertEqual("WAITING_FOR_GPS_GATE_REANCHOR", self.recovery_states[-1])
        self.assertEqual(initial_public_count, len(self.public_poses))

        # gate 적용 ack까지 받아야만 GPS reset 복구를 공개한다.
        self.ack_publisher.publish(copy.deepcopy(self.reanchors[-1]))
        self.assertTrue(self.public_pose_event.wait(1.0))
        self.assertEqual(initial_public_count + 1, len(self.public_poses))
        self.assertAlmostEqual(10.0, self.public_poses[-1].pose.pose.position.x)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.recovery_states and self.recovery_states[-1] == "RECOVERED_WITH_GPS_RESET":
                break
            rospy.sleep(0.02)
        self.assertEqual("RECOVERED_WITH_GPS_RESET", self.recovery_states[-1])


if __name__ == "__main__":
    rospy.init_node("test_gps_only_relocalization")
    rostest.rosrun(
        "mando_localization",
        "test_gps_only_relocalization",
        GpsOnlyRelocalizationTest,
    )
