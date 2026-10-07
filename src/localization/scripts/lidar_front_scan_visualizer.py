#!/usr/bin/env python3
"""차량 전방을 기준으로 LiDAR 표시 범위를 제한한다. 센서 TF는 발행하지 않는다."""

import copy
import math

import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan


def mask_front_sector(scan, rotation, min_angle_rad, max_angle_rad):
    """센서 원점에서 본 ray 방향을 차량 축으로 회전한 뒤 후방을 NaN 처리한다.

    배열을 자르거나 재정렬하지 않아 beam timestamp와 각도 인덱스가 유지된다.
    장착 translation은 방향 판정에 쓰지 않는다. 센서보다 뒤쪽이 제거 기준이다.
    """
    if (not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0.0):
        raise ValueError("invalid scan angles")
    if not (-math.pi <= min_angle_rad < max_angle_rad <= math.pi):
        raise ValueError("invalid front sector")
    if scan.intensities and len(scan.intensities) != len(scan.ranges):
        raise ValueError("intensity count differs from range count")
    values = (rotation.x, rotation.y, rotation.z, rotation.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid mounting quaternion")
    x, y, z, w = (value / norm for value in values)
    r00, r01 = 1 - 2 * (y * y + z * z), 2 * (x * y - w * z)
    r10, r11 = 2 * (x * y + w * z), 1 - 2 * (x * x + z * z)
    output = copy.deepcopy(scan)
    output.ranges = list(scan.ranges)
    output.intensities = list(scan.intensities)
    # LaserScan float32 각도에서 +/-90도 경계의 반올림 오차를 허용한다.
    epsilon = 1e-7
    for index in range(len(scan.ranges)):
        angle = scan.angle_min + index * scan.angle_increment
        cosine, sine = math.cos(angle), math.sin(angle)
        forward = r00 * cosine + r01 * sine
        left = r10 * cosine + r11 * sine
        vehicle_angle = math.atan2(left, forward)
        keep = (math.hypot(forward, left) > 1e-9
                and min_angle_rad - epsilon <= vehicle_angle <= max_angle_rad + epsilon)
        if not keep:
            output.ranges[index] = float("nan")
            if output.intensities:
                output.intensities[index] = 0.0
    return output


class LidarFrontScanVisualizer:
    def __init__(self):
        self._input = rospy.get_param("~topics/lidar_scan")
        self._output = rospy.get_param("~output_topic")
        self._base_frame = rospy.get_param("~frames/base_link")
        self._lidar_frame = rospy.get_param("~frames/lidar")
        self._minimum = math.radians(float(rospy.get_param("~sector/min_angle_deg")))
        self._maximum = math.radians(float(rospy.get_param("~sector/max_angle_deg")))
        self._timeout = float(rospy.get_param("~tf_timeout_sec"))
        if rospy.resolve_name(self._input) == rospy.resolve_name(self._output):
            raise ValueError("scan input and output must differ")
        if not (-math.pi <= self._minimum < self._maximum <= math.pi):
            raise ValueError("invalid front sector")
        if not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError("tf_timeout_sec must be finite and positive")
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._publisher = rospy.Publisher(self._output, LaserScan, queue_size=2)
        self._subscriber = rospy.Subscriber(
            self._input, LaserScan, self._callback, queue_size=2
        )
        rospy.loginfo("LiDAR 전방 표시: %s -> %s, 차량 기준 %.1f~%.1f deg",
                      self._input, self._output,
                      math.degrees(self._minimum), math.degrees(self._maximum))

    def _callback(self, scan):
        if scan.header.frame_id != self._lidar_frame or scan.header.stamp.is_zero():
            rospy.logwarn_throttle(5.0, "LiDAR 표시 생략: frame 또는 timestamp 불일치")
            return
        try:
            transform = self._buffer.lookup_transform(
                self._base_frame, scan.header.frame_id, scan.header.stamp,
                rospy.Duration(self._timeout)
            )
            masked = mask_front_sector(
                scan, transform.transform.rotation, self._minimum, self._maximum
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError) as error:
            rospy.logwarn_throttle(5.0, "LiDAR 표시 생략: %s", error)
            return
        self._publisher.publish(masked)


if __name__ == "__main__":
    rospy.init_node("lidar_front_scan_visualizer")
    LidarFrontScanVisualizer()
    rospy.spin()
