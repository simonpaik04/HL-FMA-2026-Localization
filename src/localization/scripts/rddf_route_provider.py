#!/usr/bin/env python3
"""Publish the localization-owned RDDF catalogue, without choosing a mission."""
import csv
import json
import math
from pathlib import Path


def load_catalogue(directory):
    directory = Path(directory)
    with (directory / 'yongin_route_project.json').open(encoding='utf-8') as stream:
        metadata = json.load(stream)
    routes = []
    names = set()
    for path in sorted(directory.glob('yongin_*.csv')):
        with path.open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) < 2:
            raise ValueError('RDDF needs at least two points: ' + str(path))
        name = rows[0]['route_name']
        if name in names or any(row['route_name'] != name for row in rows):
            raise ValueError('Duplicate or mixed RDDF name: ' + name)
        names.add(name)
        points = []
        for index, row in enumerate(rows):
            point = tuple(float(row[key]) for key in ('east_m', 'north_m', 'path_yaw_rad'))
            if int(row['index']) != index or not all(math.isfinite(v) for v in point):
                raise ValueError('Invalid RDDF point: ' + name)
            points.append(point)
        direction = -1 if metadata.get('route_directions', {}).get(name) == 'reverse' else 1
        routes.append((name, direction, points))
    return metadata['origin'], routes


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from planning_interfaces.msg import Route, RouteMap
    rospy.init_node('rddf_route_provider')
    origin, catalogue = load_catalogue(rospy.get_param('~rddf_directory'))
    message = RouteMap()
    message.header.frame_id = rospy.get_param('~frame_id', 'map')
    message.header.stamp = rospy.Time.now()
    message.origin_latitude = origin['lat']
    message.origin_longitude = origin['lng']
    for name, direction, points in catalogue:
        route = Route()
        route.name, route.direction = name, direction
        route.section = int(name.split('_')[0].split('-')[0])
        route.path.header = message.header
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = x, y
            # RDDF quaternions are movement tangents, not a parking gear plan.
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            route.path.poses.append(pose)
        message.routes.append(route)
    publisher = rospy.Publisher('/route/map', RouteMap, queue_size=1, latch=True)
    publisher.publish(message)
    rospy.loginfo('Published %d RDDF routes', len(message.routes))
    rospy.spin()


if __name__ == '__main__':
    main()
