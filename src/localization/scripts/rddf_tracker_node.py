#!/usr/bin/env python3
"""Publish current RDDF identification independently of RViz and screen seeking."""
import math
from pathlib import Path
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mando_localization.msg import RddfCandidate, RddfMatch
from rddf_initialization_core import RddfRouteMap
from rddf_tracking_core import RddfTracker


def candidate_message(candidate):
    result = RddfCandidate()
    result.route_name = candidate['route']
    result.source_route_name = candidate['source_route']
    result.segment_index = candidate['index']
    result.projected_point.x = candidate['x']
    result.projected_point.y = candidate['y']
    result.distance_m = candidate['distance']
    result.heading_rad = candidate['yaw']
    result.segment_fraction = candidate['fraction']
    return result


def match_message(result, stamp, pose_stamp, frame):
    message = RddfMatch()
    message.header.stamp = stamp
    message.header.frame_id = frame
    message.pose_stamp = pose_stamp
    message.matched = result['accepted']
    message.reason = result['reason']
    message.route_name = result['route'] if message.matched else ''
    message.source_route_name = result['source_route'] if message.matched else ''
    message.segment_index = result['index'] if message.matched else -1
    message.has_nearest = 'distance' in result
    if message.has_nearest:
        message.nearest = candidate_message(result)
    message.candidates = [candidate_message(item) for item in result.get('candidates', [])]
    return message


class RddfTrackerNode:
    def __init__(self):
        root = Path(rospy.get_param('~package_directory'))
        directory = Path(rospy.get_param('~initialization/rddf_directory'))
        routes = RddfRouteMap(directory if directory.is_absolute() else root/directory)
        config = rospy.get_param('~tracking')
        rate = float(config.pop('publish_rate_hz'))
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be finite and positive')
        self.tracker = RddfTracker(routes, rospy.get_param('~frames/map'), **config)
        self.lock = threading.Lock()
        self.pose_stamp = rospy.Time()
        topics = rospy.get_param('~topics')
        self.publisher = rospy.Publisher(topics['current_rddf'], RddfMatch, queue_size=1, latch=False)
        self.subscribers = [
            rospy.Subscriber(topics['global_odometry'], Odometry, self.pose_callback, queue_size=1),
            rospy.Subscriber(topics['valid'], Bool, self.valid_callback, queue_size=1),
        ]
        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.publish, reset=True)

    def pose_callback(self, message):
        with self.lock:
            point = message.pose.pose.position
            self.tracker.update_pose(point.x, point.y, message.header.stamp.to_sec(),
                                     rospy.Time.now().to_sec(), message.header.frame_id)
            # Keep the exact ROS timestamp for downstream joins; epoch seconds
            # as float cannot preserve every nanosecond in the input header.
            self.pose_stamp = message.header.stamp

    def valid_callback(self, message):
        with self.lock:
            self.tracker.update_valid(message.data, rospy.Time.now().to_sec())

    def publish(self, _event):
        with self.lock:
            stamp = rospy.Time.now()
            result = self.tracker.evaluate(stamp.to_sec())
            pose = self.tracker.pose
            pose_stamp = self.pose_stamp if pose else rospy.Time()
            self.publisher.publish(match_message(result, stamp, pose_stamp, self.tracker.map_frame))


if __name__ == '__main__':
    rospy.init_node('rddf_tracker')
    RddfTrackerNode()
    rospy.spin()
