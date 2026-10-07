#!/usr/bin/python3
"""Synthetic startup tests through production normalizer, initializer and EKFs.

Both launches use an isolated rostest master. The GPS launch injects clock
approval and NavSatFix; the manual launch supplies neither GPS nor clock approval.
No claim about physical GNSS/PPS, IMU accuracy, or GUI pointer events is made.
"""
import csv
import json
import math
from pathlib import Path
import threading
import time
import unittest

import rospkg
import rospy
import rostest
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, String
from mando_localization.srv import SetInitialHeading


EPOCH = 1800000000.0
PREFIX = "/mando_localization/internal/initialization/"


def yaw_of(quaternion):
    q = quaternion
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_error(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class RddfInitializationRosTest(unittest.TestCase):
    def test_startup_selection_and_confirmed_pose(self):
        self.mode = rospy.get_param("~mode")
        self.assertIn(self.mode, ("gps", "manual"))
        self.tick = 0
        self.alive = 0
        self.lock = threading.RLock()
        self.received = {key: [] for key in ("normalized", "calibrated", "local", "global", "output", "ready", "status", "committed", "gps_approved")}

        def store(key, message):
            with self.lock:
                self.received[key].append((rospy.Time.now().to_sec(), message))

        self.subscribers = [
            rospy.Subscriber(topic, kind, lambda message, key=key: store(key, message), queue_size=500)
            for key, topic, kind in (
                ("normalized", "/molit/localization/imu/normalized", Imu),
                ("calibrated", "/molit/localization/imu/calibrated", Imu),
                ("local", "/molit/localization/local/odometry", Odometry),
                ("global", "/molit/localization/global/odometry", Odometry),
                ("output", "/molit/localization/odometry", Odometry),
                ("ready", PREFIX + "ready", Bool),
                ("status", PREFIX + "status", String),
                ("committed", PREFIX + "committed_pose", PoseWithCovarianceStamped),
                ("gps_approved", "/mando_localization/internal/gps/gate_pose", PoseWithCovarianceStamped),
            )
        ]
        self.clock = rospy.Publisher("/clock", Clock, queue_size=10, latch=True)
        self.imu = rospy.Publisher("/molit/sensors/imu/data", Imu, queue_size=100)
        self.feedback = rospy.Publisher("/erp42_serial/feedback", SerialFeedBack, queue_size=10)
        self.clock_ready = rospy.Publisher("/mando_localization/internal/timing/clock_ready", Bool, queue_size=10)
        self.gps = rospy.Publisher("/molit/sensors/gps/fix", NavSatFix, queue_size=10)
        self.manual = rospy.Publisher(PREFIX + "manual_request", String, queue_size=1)

        package = Path(rospkg.RosPack().get_path("mando_localization"))
        directory = package / "rddf"
        route = "7" if self.mode == "gps" else "2"
        segment_index = 192 if self.mode == "gps" else 100
        with (directory / ("yongin_" + route + ".csv")).open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        start, end = rows[segment_index], rows[segment_index + 1]
        sx, sy, ex, ey = [float(row[key]) for row in (start, end) for key in ("east_m", "north_m")]
        target = {"route": route, "x": (sx + ex) / 2, "y": (sy + ey) / 2,
                  "yaw": math.atan2(ey - sy, ex - sx)}
        origin = json.loads((directory / "yongin_route_project.json").read_text())["origin"]
        latitude = math.radians(origin["lat"])
        d = math.sqrt(1 - 6.69437999014e-3 * math.sin(latitude) ** 2)
        # Synthetic fix is the GPS antenna, 0.65 m ahead of the rear-axle base_link.
        lever = rospy.get_param("/rddf_initializer/lever_arm")
        antenna_x = target["x"] + math.cos(target["yaw"]) * lever["x_m"] - math.sin(target["yaw"]) * lever["y_m"]
        antenna_y = target["y"] + math.sin(target["yaw"]) * lever["x_m"] + math.cos(target["yaw"]) * lever["y_m"]
        self.fix_lat = origin["lat"] + math.degrees(antenna_y / (6378137.0 * (1 - 6.69437999014e-3) / d ** 3))
        self.fix_lon = origin["lng"] + math.degrees(antenna_x / (6378137.0 / d * math.cos(latitude)))

        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.clock.publish(Clock(rospy.Time.from_sec(EPOCH)))
            if all(pub.get_num_connections() for pub in (self.imu, self.feedback, self.clock_ready, self.gps, self.manual)):
                break
            time.sleep(0.02)
        self.assertTrue(all(pub.get_num_connections() for pub in
                            (self.imu, self.feedback, self.clock_ready, self.gps, self.manual)),
                        "startup subscribers did not connect")
        self.pump(2.0)
        self.assertGreater(len(self.received["normalized"]), 100)
        self.assert_waiting()

        if self.mode == "gps":
            # Neither invalid fixes nor a moving vehicle may choose the initial pose.
            self.pump(0.5, gps_status=-1)
            self.assert_waiting()
            self.pump(0.6, gps_status=0, speed=1.0)
            self.assert_waiting()
            self.pump(0.3)
        else:
            self.manual.publish(String(json.dumps(dict(target, frame_id="map", stamp=EPOCH - 20))))
            self.pump(0.3)
            self.assert_waiting()
            for invalid_index in (-1, True, 999999):
                self.manual.publish(String(json.dumps(dict(target, frame_id="map",
                    stamp=rospy.Time.now().to_sec(), index=invalid_index))))
                self.pump(0.2)
                self.assert_waiting()
            request = dict(target, frame_id="map", stamp=rospy.Time.now().to_sec())
            request["index"] = segment_index
            request["yaw"] = 0.0  # Deliberately wrong; RDDF tangent must be authoritative.
            self.manual.publish(String(json.dumps(request)))

        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not self.is_ready():
            self.pump(0.1, gps_status=0 if self.mode == "gps" else None)
        self.assertTrue(self.is_ready(), "initializer did not reach READY: " + self.last_status())
        self.pump(1.0, gps_status=0 if self.mode == "gps" else None)
        stationary_metrics = None
        if self.mode == "manual":
            # Prove the complete stationary contract after the RDDF pose is
            # committed: fresh zero encoder feedback must hold both EKFs at
            # the selected position while rejecting a large AHRS yaw drift.
            hold_start = EPOCH + self.tick / 100.0
            self.pump(4.0, imu_yaw_rate_degps=20.0)
            time.sleep(0.2)
        with self.lock:
            data = {key: list(value) for key, value in self.received.items()}
        if self.mode == "manual":
            settled_after = hold_start + 0.2
            stationary = {
                key: [message for _, message in data[key]
                      if message.header.stamp.to_sec() >= settled_after]
                for key in ("normalized", "calibrated", "local", "global", "output")
            }
            self.assertGreater(len(stationary["normalized"]), 300)
            raw_yaw_change = math.degrees(angle_error(
                yaw_of(stationary["normalized"][-1].orientation),
                yaw_of(stationary["normalized"][0].orientation)))
            self.assertGreater(raw_yaw_change, 70.0)
            self.assertGreater(len(stationary["calibrated"]), 300)
            calibrated_yaw_error = max(math.degrees(angle_error(
                yaw_of(message.orientation), target["yaw"]))
                for message in stationary["calibrated"])
            self.assertLess(calibrated_yaw_error, 0.01)
            self.assertTrue(all(abs(message.angular_velocity.z) < 1e-12
                                for message in stationary["calibrated"]))

            stationary_metrics = {}
            for name in ("local", "global", "output"):
                messages = stationary[name]
                # The public output is intentionally cut after the no-GPS
                # dead-reckoning budget; validate every sample that the safety
                # gate allows instead of requiring it for the full 4 seconds.
                minimum_samples = 20 if name == "output" else 50
                self.assertGreaterEqual(len(messages), minimum_samples,
                                        name + " stationary output missing")
                first = messages[0].pose.pose.position
                max_target_error = max(math.hypot(
                    message.pose.pose.position.x - target["x"],
                    message.pose.pose.position.y - target["y"]) for message in messages)
                max_position_drift = max(math.hypot(
                    message.pose.pose.position.x - first.x,
                    message.pose.pose.position.y - first.y) for message in messages)
                max_yaw_error = max(math.degrees(angle_error(
                    yaw_of(message.pose.pose.orientation), target["yaw"])) for message in messages)
                initial_yaw = yaw_of(messages[0].pose.pose.orientation)
                max_yaw_span = max(math.degrees(angle_error(
                    yaw_of(message.pose.pose.orientation), initial_yaw)) for message in messages)
                self.assertLess(max_target_error, 0.15, name)
                self.assertLess(max_position_drift, 0.05, name)
                self.assertLess(max_yaw_error, 1.0, name)
                self.assertLess(max_yaw_span, 0.2, name)
                stationary_metrics[name] = (max_position_drift, max_yaw_span)
        self.assertTrue(data["calibrated"])
        first_ready = next(stamp for stamp, message in data["ready"] if message.data)
        confirmation_samples = rospy.get_param("/rddf_initializer/initialization/confirmation_samples")
        for name, expected_frame in (("local", "odom"), ("global", "map")):
            observations = [message for _, message in data[name]
                            if message.header.stamp.to_sec() <= first_ready + 0.02
                            and math.hypot(message.pose.pose.position.x - target["x"],
                                           message.pose.pose.position.y - target["y"]) < 0.15
                            and angle_error(yaw_of(message.pose.pose.orientation), target["yaw"]) < math.radians(1)]
            self.assertGreaterEqual(len({m.header.stamp.to_nsec() for m in observations}), confirmation_samples,
                                    name + " was not confirmed before READY")
            latest = data[name][-1][1]
            self.assertEqual(latest.header.frame_id, expected_frame)
            self.assertEqual(latest.child_frame_id, "base_link")
            self.assertLess(math.hypot(latest.pose.pose.position.x - target["x"],
                                       latest.pose.pose.position.y - target["y"]), 0.15)
            self.assertLess(angle_error(yaw_of(latest.pose.pose.orientation), target["yaw"]), math.radians(1))
        self.assertEqual(len(data["committed"]), 1)
        commit = data["committed"][0][1]
        self.assertEqual(commit.header.frame_id, "map")
        self.assertLess(math.hypot(commit.pose.pose.position.x - target["x"],
                                  commit.pose.pose.position.y - target["y"]), 0.05)
        self.assertLess(angle_error(yaw_of(data["calibrated"][-1][1].orientation), target["yaw"]), 0.001)
        source = "GPS_RDDF" if self.mode == "gps" else "MANUAL_RDDF"
        self.assertEqual(json.loads(self.last_status())["source"], source)
        self.assertTrue(data["output"], "initialized session produced no valid public odometry")
        self.assertTrue(all(stamp >= first_ready - 0.02 for stamp, _ in data["output"]),
                        "public odometry was emitted before initialization confirmation")
        if self.mode == "gps":
            self.assertGreaterEqual(len(data["gps_approved"]), 3,
                                    "GPS gate did not continue approving fixes after initialization")
            for stamp, message in data["gps_approved"]:
                self.assertGreaterEqual(stamp, first_ready - 0.02)
                self.assertEqual(message.header.frame_id, "map")
                self.assertLess(math.hypot(message.pose.pose.position.x - target["x"],
                                           message.pose.pose.position.y - target["y"]), 0.15)
        else:
            self.assertFalse(data["gps_approved"])

        # A later click cannot overwrite a committed session's initial state.
        self.manual.publish(String(json.dumps({"frame_id": "map", "stamp": rospy.Time.now().to_sec(),
                                               "route": "1_right", "x": 37.006811, "y": 26.908526})))
        replacement = rospy.ServiceProxy("/calibrated_imu/set_initial_heading", SetInitialHeading)(2, 0.0, "replacement", 10.0)
        self.assertFalse(replacement.accepted)
        self.pump(0.3)
        self.assertEqual(len(self.received["committed"]), 1)
        self.assertEqual(json.loads(self.last_status())["route"], route)
        self.assertTrue(self.is_ready())
        if stationary_metrics is not None:
            print("encoder zero + 80 deg raw yaw drift: calibrated error %.6f deg; "
                  "Local drift %.6f m/yaw span %.6f deg; Global drift %.6f m/yaw span %.6f deg; "
                  "public drift %.6f m/yaw span %.6f deg"
                  % (calibrated_yaw_error,
                     stationary_metrics["local"][0], stationary_metrics["local"][1],
                     stationary_metrics["global"][0], stationary_metrics["global"][1],
                     stationary_metrics["output"][0], stationary_metrics["output"][1]))
        print("%s: waited without calibrated/output, selected %s at (%.6f, %.6f), yaw %.6f deg; both EKFs confirmed before READY"
              % (self.mode, route, target["x"], target["y"], math.degrees(target["yaw"])))

    def last_status(self):
        with self.lock:
            return self.received["status"][-1][1].data if self.received["status"] else "{}"

    def is_ready(self):
        with self.lock:
            return bool(self.received["ready"] and self.received["ready"][-1][1].data)

    def assert_waiting(self):
        with self.lock:
            self.assertTrue(self.received["ready"], "ready heartbeat missing")
            self.assertFalse(any(message.data for _, message in self.received["ready"]))
            self.assertFalse(self.received["calibrated"], self.last_status())
            self.assertFalse(self.received["output"], "public odometry leaked before selection")
            self.assertFalse(self.received["gps_approved"], "GPS approval leaked before initialization")

    def pump(self, seconds, gps_status=None, speed=0.0, imu_yaw_rate_degps=0.0):
        start_tick = self.tick
        for _ in range(int(round(seconds * 100))):
            stamp = rospy.Time.from_sec(EPOCH + self.tick / 100)
            self.clock.publish(Clock(stamp))
            message = Imu()
            message.header.stamp, message.header.frame_id = stamp, "imu_link"
            imu_yaw_deg = 99.58 + imu_yaw_rate_degps * (self.tick - start_tick) / 100.0
            message.orientation.z = math.sin(math.radians(imu_yaw_deg) / 2)
            message.orientation.w = math.cos(math.radians(imu_yaw_deg) / 2)
            message.angular_velocity.z = math.radians(imu_yaw_rate_degps)
            message.linear_acceleration.z = 9.81
            self.imu.publish(message)
            if self.tick % 5 == 0:
                feedback = SerialFeedBack()
                feedback.alive, self.alive = self.alive, (self.alive + 1) % 256
                feedback.speed, feedback.encoder = speed, 100 if speed else 0
                self.feedback.publish(feedback)
                self.clock_ready.publish(Bool(self.mode == "gps"))
            if gps_status is not None and self.tick % 10 == 0:
                fix = NavSatFix()
                fix.header.stamp, fix.header.frame_id = stamp, "gps_link"
                fix.status.status = gps_status
                fix.latitude, fix.longitude, fix.altitude = self.fix_lat, self.fix_lon, 0.0
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                fix.position_covariance = [0.09, 0, 0, 0, 0.09, 0, 0, 0, 1.0]
                self.gps.publish(fix)
            self.tick += 1
            time.sleep(0.006)


if __name__ == "__main__":
    rospy.init_node("test_rddf_initialization")
    rostest.rosrun("mando_localization", "rddf_initialization", RddfInitializationRosTest)
