#!/usr/bin/env python3
"""Read-only RDDF geometry for startup initialization and map-click preview.

XY is east/north in metres relative to the project's WGS84 origin. Yaw is the
vehicle body heading, east=0 and counterclockwise positive. Reverse routes
face opposite the geometric segment tangent.
The returned pose assumes the physical vehicle follows the configured direction;
position alone cannot resolve intersecting roads or opposite travel directions.
"""
import csv
import json
import math
from pathlib import Path

import numpy as np


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
CSV_HEADER = [
    "route_id", "route_name", "closed", "index", "latitude", "longitude",
    "east_m", "north_m", "distance_m", "path_yaw_rad",
]


def _wgs84(latitude, longitude):
    try:
        latitude, longitude = float(latitude), float(longitude)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("WGS84 coordinates must be finite numbers") from error
    if not (math.isfinite(latitude) and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
        raise ValueError("invalid WGS84 latitude or longitude")
    return latitude, longitude


def _angle_difference(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class RddfRouteMap:
    """Load project origin and exported CSVs without ROS or modifying sources.

    ``routes`` maps CSV route names to immutable Nx2 XY arrays. ``match`` returns
    ordinary JSON-serializable dictionaries. Rejections retain the nearest pose
    when available, exclusively for a rejected/ambiguous UI preview.

    ``same_branch_distance_m`` and ``same_branch_heading_deg`` are equivalence
    tolerances for overlapping exports, not claimed GPS measurement accuracy.
    """

    def __init__(self, directory, same_branch_distance_m=0.10,
                 same_branch_heading_deg=20.0):
        self.directory = Path(directory).expanduser().resolve()
        self.same_branch_distance_m = float(same_branch_distance_m)
        self.same_branch_heading_rad = math.radians(float(same_branch_heading_deg))
        if not (math.isfinite(self.same_branch_distance_m)
                and self.same_branch_distance_m >= 0.0
                and math.isfinite(self.same_branch_heading_rad)
                and 0.0 <= self.same_branch_heading_rad < math.pi / 2):
            raise ValueError("invalid RDDF branch equivalence tolerances")
        with (self.directory / "yongin_route_project.json").open(encoding="utf-8-sig") as stream:
            project = json.load(stream)
        try:
            lat, lng = _wgs84(project["origin"]["lat"], project["origin"]["lng"])
        except (KeyError, TypeError) as error:
            raise ValueError("RDDF project needs origin.lat and origin.lng") from error
        self.origin = {"lat": lat, "lng": lng}
        self.route_directions = project.get("route_directions", {})
        if (not isinstance(self.route_directions, dict)
                or any(value not in ("forward", "reverse")
                       for value in self.route_directions.values())):
            raise ValueError("RDDF route_directions must map names to forward or reverse")
        self.routes = {}
        self._segments = []
        self._adjacent = []
        for path in sorted(self.directory.glob("*.csv")):
            self._load_route(path)
        unknown = set(self.route_directions) - set(self.routes)
        if unknown:
            raise ValueError("RDDF direction references unknown routes: {}".format(sorted(unknown)))
        if not self._segments:
            raise ValueError("RDDF directory has no nondegenerate routes")
        self.route_groups = project.get('route_groups', {})
        self.route_group_names = {}
        if not isinstance(self.route_groups, dict):
            raise ValueError('RDDF route_groups must map group names to CSV route names')
        for name, members in self.route_groups.items():
            if (not isinstance(name, str) or not name.strip() or name in self.routes
                    or not isinstance(members, list) or not members):
                raise ValueError('invalid RDDF route group: {}'.format(name))
            for member in members:
                if (not isinstance(member, str) or member not in self.routes
                        or member in self.route_group_names):
                    raise ValueError('unknown or duplicated RDDF group member: {}'.format(member))
                self.route_group_names[member] = name
        self._starts = np.asarray([segment["start"] for segment in self._segments])
        self._vectors = np.asarray([segment["vector"] for segment in self._segments])
        self._length_squared = np.sum(self._vectors * self._vectors, axis=1)

    def _load_route(self, path):
        with path.open(encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != CSV_HEADER:
                raise ValueError("unexpected RDDF CSV header: {}".format(path))
            rows = list(reader)
        if len(rows) < 2:
            raise ValueError("RDDF route needs at least two points: {}".format(path))
        name, route_id = rows[0]["route_name"], rows[0]["route_id"]
        if not name or name in self.routes:
            raise ValueError("RDDF route name is empty or duplicated: {}".format(path))
        closed_text = rows[0]["closed"].lower()
        if closed_text not in ("false", "true", "0", "1"):
            raise ValueError("invalid RDDF closed flag: {}".format(path))
        closed = closed_text in ("true", "1")
        points = []
        indices = []
        previous_distance = -math.inf
        try:
            for row in rows:
                if (row["route_name"] != name or row["route_id"] != route_id
                        or row["closed"].lower() != closed_text):
                    raise ValueError("route identity changes within CSV")
                index = int(row["index"])
                if index < 0 or (indices and index != indices[-1] + 1):
                    raise ValueError("route indices must be consecutive and nonnegative")
                _wgs84(row["latitude"], row["longitude"])
                values = [float(row[key]) for key in
                          ("east_m", "north_m", "distance_m", "path_yaw_rad")]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("route values must be finite")
                if values[2] < 0.0 or values[2] < previous_distance:
                    raise ValueError("route distances must be nonnegative and monotonic")
                previous_distance = values[2]
                points.append(values[:2])
                indices.append(index)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError("invalid RDDF data in {}: {}".format(path, error)) from error
        array = np.asarray(points, dtype=float)
        array.setflags(write=False)
        self.routes[name] = array
        segment_ids = []
        for index in range(len(points) if closed else len(points) - 1):
            start = array[index]
            with np.errstate(over="ignore", invalid="ignore"):
                vector = array[(index + 1) % len(points)] - start
                length_squared = float(np.dot(vector, vector))
            if not math.isfinite(length_squared):
                raise ValueError("RDDF segment exceeds finite metric range: {}".format(path))
            if length_squared <= 1e-16:
                continue
            yaw = math.atan2(vector[1], vector[0])
            if self.route_directions.get(name, "forward") == "reverse":
                yaw = math.atan2(-vector[1], -vector[0])
            segment_ids.append(len(self._segments))
            self._segments.append({
                "route": name, "index": indices[index], "start": start,
                "vector": vector, "yaw": yaw,
            })
        if not segment_ids:
            raise ValueError("RDDF route has no nonzero segment: {}".format(path))
        self._adjacent.extend(zip(segment_ids[:-1], segment_ids[1:]))
        if closed and len(segment_ids) > 1:
            self._adjacent.append((segment_ids[-1], segment_ids[0]))

    def project_gps(self, latitude, longitude):
        """Use the same local WGS84 ellipsoid approximation as the current viewer."""
        latitude, longitude = _wgs84(latitude, longitude)
        origin_lat = math.radians(self.origin["lat"])
        denominator = math.sqrt(1.0 - WGS84_E2 * math.sin(origin_lat) ** 2)
        east = (WGS84_A_M / denominator * math.cos(origin_lat)
                * math.radians(longitude - self.origin["lng"]))
        north = (WGS84_A_M * (1.0 - WGS84_E2) / denominator ** 3
                 * math.radians(latitude - self.origin["lat"]))
        return east, north

    def match(self, x, y, max_distance_m, ambiguity_distance_m=0.5, route_name=None,
              segment_index=None):
        """Snap to a segment, rejecting similarly close incompatible branches.

        Candidates within ``best distance + ambiguity_distance_m`` compete, up
        to ``max_distance_m``. Adjacent forward segments of one route form one
        physical branch, so dense samples and ordinary corners do not produce
        false ambiguity. Reverse/retraced segments still compete. Route filtering
        does not disable detection of a self-intersection inside that route.
        An explicit manual route/segment choice resolves that ambiguity; the
        position and heading are still projected from the selected RDDF segment.
        """
        try:
            x, y, maximum, margin = map(float, (x, y, max_distance_m, ambiguity_distance_m))
        except (TypeError, ValueError, OverflowError):
            return {"accepted": False, "reason": "INVALID_INPUT"}
        if not all(math.isfinite(value) for value in (x, y, maximum, margin)) or min(maximum, margin) < 0:
            return {"accepted": False, "reason": "INVALID_INPUT"}
        if route_name is not None and (not isinstance(route_name, str) or route_name not in self.routes):
            return {"accepted": False, "reason": "UNKNOWN_ROUTE"}
        if segment_index is not None:
            if (route_name is None or isinstance(segment_index, bool)
                    or not isinstance(segment_index, int) or segment_index < 0):
                return {"accepted": False, "reason": "INVALID_SEGMENT"}
            selected = [i for i, segment in enumerate(self._segments)
                        if segment["route"] == route_name and segment["index"] == segment_index]
            if not selected:
                return {"accepted": False, "reason": "UNKNOWN_SEGMENT"}
        with np.errstate(over="ignore", invalid="ignore"):
            vector_to_query = np.asarray((x, y)) - self._starts
            fractions = np.clip(np.sum(vector_to_query * self._vectors, axis=1)
                                / self._length_squared, 0.0, 1.0)
            projected = self._starts + fractions[:, None] * self._vectors
            distances = np.linalg.norm(projected - (x, y), axis=1)
        if not np.isfinite(distances).all():
            return {"accepted": False, "reason": "INVALID_INPUT"}
        if route_name is not None:
            distances = np.where([s["route"] == route_name for s in self._segments],
                                 distances, np.inf)
        if segment_index is not None:
            distances = np.where(np.arange(len(distances)) == selected[0], distances, np.inf)
        best_id = int(np.argmin(distances))

        def result_for(segment_id):
            segment = self._segments[segment_id]
            return {"route": segment["route"], "index": int(segment["index"]),
                    "x": float(projected[segment_id, 0]), "y": float(projected[segment_id, 1]),
                    "yaw": float(segment["yaw"]), "distance": float(distances[segment_id]),
                    "fraction": float(fractions[segment_id])}

        best = result_for(best_id)
        result = dict(best, accepted=False, reason="TOO_FAR")
        if best["distance"] > maximum:
            return result

        near = np.flatnonzero(distances <= min(maximum, best["distance"] + margin) + 1e-9)
        parents = {int(index): int(index) for index in near}

        def root(index):
            while parents[index] != index:
                parents[index] = parents[parents[index]]
                index = parents[index]
            return index

        for first, second in self._adjacent:
            if first in parents and second in parents:
                if _angle_difference(self._segments[first]["yaw"], self._segments[second]["yaw"]) <= math.pi / 2 + 1e-9:
                    parents[root(second)] = root(first)
        groups = {}
        for segment_id in near:
            segment_id = int(segment_id)
            group = root(segment_id)
            if group not in groups or distances[segment_id] < distances[groups[group]]:
                groups[group] = segment_id
        representatives = sorted(groups.values(), key=lambda index: (distances[index], index))
        candidates = [result_for(index) for index in representatives]
        competing = []
        for candidate in candidates:
            same_direction = _angle_difference(best["yaw"], candidate["yaw"]) <= self.same_branch_heading_rad
            same_point = math.hypot(best["x"] - candidate["x"], best["y"] - candidate["y"]) <= self.same_branch_distance_m
            if not (same_direction and same_point):
                competing.append(candidate)
        result.update(accepted=not competing, reason="AMBIGUOUS_ROUTE" if competing else "MATCHED",
                      candidate_count=len(candidates), candidates=candidates)
        return result
