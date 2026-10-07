#!/usr/bin/env python3
"""Shared RDDF route identification and live-input freshness checks, without ROS."""
import math


def current_rddf_members(route_map, names):
    """Expand display groups to separate CSV polylines without joining endpoints."""
    return list(dict.fromkeys(member for name in names
                             for member in route_map.route_groups.get(name, [name])))


def current_rddf_match(route_map, position, localization_valid, max_distance_m=5.0,
                       ambiguity_distance_m=0.5):
    """Identify public RDDF groups while preserving each source segment's geometry."""
    if position is None:
        return dict(accepted=False, reason='NO_FRESH_GLOBAL', routes=[],
                    text='RDDF: waiting for Global position')
    if not localization_valid:
        return dict(accepted=False, reason='LOCALIZATION_INVALID', routes=[],
                    text='RDDF: localization invalid / stale')
    result = route_map.match(position[0], position[1], max_distance_m,
                             ambiguity_distance_m)
    def grouped(candidate):
        source = candidate['route']
        return dict(candidate, source_route=source,
                    route=route_map.route_group_names.get(source, source))
    if 'route' in result:
        result = grouped(result)
    if 'candidates' in result:
        result['candidates'] = [grouped(candidate) for candidate in result['candidates']]
    names = sorted({candidate['route'] for candidate in result.get('candidates', [])})
    if len(names) > 1:
        result.update(accepted=False, reason='AMBIGUOUS_ROUTE')
    elif (len(names) == 1 and names[0] in route_map.route_groups
          and result['reason'] == 'AMBIGUOUS_ROUTE'):
        # Different in/out legs still identify one parking group. Their source
        # segments/headings remain separate candidates, including at reversals.
        result.update(accepted=True, reason='MATCHED')
    result['routes'] = names
    if result['accepted']:
        result['text'] = 'RDDF: {} | {:.2f} m'.format(result['route'], result['distance'])
    elif result['reason'] == 'AMBIGUOUS_ROUTE':
        result['text'] = 'RDDF candidates: {} | {:.2f} m'.format(' / '.join(names), result['distance'])
    elif result['reason'] == 'TOO_FAR':
        result['text'] = 'RDDF: off route | nearest {}: {:.2f} m'.format(result['route'], result['distance'])
    else:
        result['text'] = 'RDDF: position unavailable'
    return result


class RddfTracker:
    """Match only fresh, valid Global positions in the RDDF map frame."""
    def __init__(self, route_map, map_frame='map', max_distance_m=5.0,
                 ambiguity_distance_m=0.5, pose_stale_sec=0.5, valid_stale_sec=1.0):
        self.route_map = route_map
        self.map_frame = map_frame.lstrip('/')
        self.maximum = float(max_distance_m)
        self.margin = float(ambiguity_distance_m)
        self.pose_stale = float(pose_stale_sec)
        self.valid_stale = float(valid_stale_sec)
        if (not self.map_frame or not all(math.isfinite(v) and v > 0 for v in
                (self.maximum, self.pose_stale, self.valid_stale))
                or not math.isfinite(self.margin) or self.margin < 0):
            raise ValueError('invalid RDDF tracker configuration')
        self.pose = self.valid = self.last_now = None

    def _observe_time(self, now):
        if self.last_now is not None and now < self.last_now:
            self.pose = self.valid = None
        self.last_now = now

    def update_pose(self, x, y, stamp, received, frame_id):
        self._observe_time(received)
        self.pose = dict(x=x, y=y, stamp=stamp, received=received, frame_id=frame_id)

    def update_valid(self, valid, received):
        self._observe_time(received)
        self.valid = (bool(valid), received)

    def evaluate(self, now):
        self._observe_time(now)
        def rejected(reason):
            return dict(accepted=False, reason=reason, routes=[])
        if self.pose is None:
            return rejected('NO_GLOBAL')
        pose = self.pose
        if not all(math.isfinite(pose[key]) for key in ('x', 'y', 'stamp', 'received')):
            return rejected('INVALID_INPUT')
        if pose['stamp'] <= 0 or pose['stamp'] > now:
            return rejected('INVALID_STAMP')
        if pose['frame_id'].lstrip('/') != self.map_frame:
            return rejected('FRAME_MISMATCH')
        if now-pose['stamp'] > self.pose_stale or now-pose['received'] > self.pose_stale:
            return rejected('STALE_GLOBAL')
        if self.valid is None or now-self.valid[1] > self.valid_stale:
            return rejected('STALE_VALID')
        if not self.valid[0]:
            return rejected('LOCALIZATION_INVALID')
        return current_rddf_match(self.route_map, (pose['x'], pose['y']), True,
                                  self.maximum, self.margin)
