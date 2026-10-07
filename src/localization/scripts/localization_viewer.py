#!/usr/bin/env python3
"""Single RViz viewer for live localization and explicitly selected recorded outputs.

Recorded outputs use a display-only common XY translation. Live RDDF-map mode
also offers an explicit startup-pose request; hovering never changes estimation.
Odometry mode displays poses without a GPS anchor or an RDDF overlay.
Time navigation redraws buffered results and never rewinds ROS /clock.
"""
import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path
import queue
import sys
import time
from typing import Dict, Tuple

# Catkin executes this script through a relay in devel/lib.  Put the real
# scripts directory first so helper modules resolve to their source files,
# rather than to Catkin relay scripts that do not export their definitions.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
from rddf_tracking_core import current_rddf_match, current_rddf_members
import rosbag
import rospkg
import yaml

WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
PACKAGE = Path(rospkg.RosPack().get_path('mando_localization'))
WORKSPACE = PACKAGE.parent.parent
FRAME = 'localization_debug'
PREFIX = '/mando_localization/visualization/debug'
DEFAULT_CONFIG = yaml.safe_load((PACKAGE/'config/localization_viewer.yaml').read_text())
TOPICS = dict(DEFAULT_CONFIG['topics'])
COLORS = {key:tuple(value) for key,value in DEFAULT_CONFIG['colors'].items()}
INITIALIZATION_PREFIX = '/mando_localization/internal/initialization'


def topdown_screen_to_map(x, y, width, height, scale, center_x, center_y,
                          angle=0., pixel_ratio=1.):
    """Invert RViz TopDownOrtho projection, including native-pixel dimensions.

    RViz RenderWidget rounds its native render-window width up to an even pixel.
    The view controller's Scale is native pixels per metre, and Angle rotates
    camera +X/+Y about map +Z. Qt event coordinates remain logical pixels.
    """
    values = [x, y, width, height, scale, center_x, center_y, angle, pixel_ratio]
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('non-finite RViz projection')
    if width <= 0 or height <= 0 or scale <= 0 or pixel_ratio <= 0:
        raise ValueError('invalid RViz projection dimensions')
    native_width = int(width * pixel_ratio)
    native_width += native_width % 2
    native_height = int(height * pixel_ratio)
    dx = (x * pixel_ratio - native_width / 2.) / scale
    dy = (native_height / 2. - y * pixel_ratio) / scale
    c, s = math.cos(angle), math.sin(angle)
    return center_x + c * dx - s * dy, center_y + s * dx + c * dy


def manual_initialization_request(match, stamp):
    """One atomic route/position request; coordinator repeats all admission checks."""
    if not match or not match.get('accepted') or not math.isfinite(stamp) or stamp <= 0:
        raise ValueError('manual initialization requires an accepted pose and valid ROS time')
    result = dict(frame_id='map', stamp=float(stamp), route=match['route'])
    if 'index' in match:
        if isinstance(match['index'], bool) or not isinstance(match['index'], int) or match['index'] < 0:
            raise ValueError('invalid manual RDDF segment')
        result['index'] = match['index']
    for key in ('x', 'y', 'yaw'):
        value = float(match[key])
        if not math.isfinite(value):
            raise ValueError('non-finite manual pose')
        result[key] = value
    return result


def scene_marker_updates(markers, known_keys):
    """Replace stable identities and retire absent objects, even after a dropped frame."""
    from visualization_msgs.msg import Marker
    keys = {(marker.ns, marker.id) for marker in markers}
    removed = []
    for namespace, ident in sorted(known_keys - keys):
        marker = Marker()
        marker.ns, marker.id, marker.action = namespace, ident, Marker.DELETE
        removed.append(marker)
    # Repeat deletions: the queue may discard the first update that removed an
    # object. There are only a fixed number of semantic marker slots per scene.
    return removed + markers, known_keys | keys


def stream_reception_status(stamps, position, stale_sec, missing_text='미수신'):
    """Return a display-only reception state for a raw sensor stream."""
    if (not math.isfinite(float(position)) or not math.isfinite(float(stale_sec))
            or stale_sec <= 0):
        raise ValueError('stream reception timing must be finite and positive')
    index = bisect.bisect_right(stamps, position)-1
    if index < 0:
        return 1, missing_text
    age = position-stamps[index]
    if not math.isfinite(age) or age < 0:
        return 2, '수신 시각 오류'
    if age <= stale_sec:
        return 0, '연결됨 · 데이터 수신 중'
    return 2, '수신 끊김'


def load_rddf(rddf_dir: Path) -> Tuple[Dict[str, np.ndarray], Dict]:
    project_path = rddf_dir / "yongin_route_project.json"
    project = json.loads(project_path.read_text(encoding="utf-8-sig"))
    routes: Dict[str, np.ndarray] = {}
    expected_header = [
        "route_id",
        "route_name",
        "closed",
        "index",
        "latitude",
        "longitude",
        "east_m",
        "north_m",
        "distance_m",
        "path_yaw_rad",
    ]
    for path in sorted(rddf_dir.glob("*.csv")):
        with path.open(encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != expected_header:
                raise RuntimeError(f"unexpected RDDF header: {path}")
            rows = list(reader)
        if not rows:
            raise RuntimeError(f"empty RDDF CSV: {path}")
        name = rows[0]["route_name"]
        routes[name] = np.asarray(
            [(float(row["east_m"]), float(row["north_m"])) for row in rows],
            dtype=float,
        )
    if not routes or not all(np.isfinite(points).all() for points in routes.values()):
        raise ValueError('RDDF routes must contain finite points')
    return routes, project

def project_wgs84(
    latitude_deg: float, longitude_deg: float, origin: Dict[str, float]
) -> Tuple[float, float]:
    latitude_rad = math.radians(float(origin["lat"]))
    sin_latitude = math.sin(latitude_rad)
    denominator = math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    prime_vertical_radius_m = WGS84_A_M / denominator
    meridian_radius_m = (
        WGS84_A_M
        * (1.0 - WGS84_E2)
        / (denominator * denominator * denominator)
    )
    east_m = (
        prime_vertical_radius_m
        * math.cos(latitude_rad)
        * math.radians(longitude_deg - float(origin["lng"]))
    )
    north_m = meridian_radius_m * math.radians(
        latitude_deg - float(origin["lat"])
    )
    return east_m, north_m

def yaw_of(q):
    values = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or abs(norm - 1.0) > 0.01:
        raise ValueError('invalid orientation quaternion')
    x, y, z, w = values / norm
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def heading_readout(current, raw_yaw=None, calibrated_yaw=None):
    """Use the displayed vehicle samples; never substitute IMU yaw for odometry."""
    headings = []
    for label, yaw in [('Raw', raw_yaw), ('Calibrated', calibrated_yaw)]:
        if yaw is not None:
            headings.append('{} yaw {:.1f}°'.format(label, math.degrees(yaw)))
    for key, label in [('local', 'Local'), ('global', 'Global')]:
        sample = current.get(key)
        headings.append('{} yaw {}'.format(label, '이 시점 표시 없음' if sample is None
                        else '{:.1f}°'.format(math.degrees(sample[2]))))
    return ' | '.join(headings)


def final_bag(path):
    path = Path(path).expanduser().resolve(strict=True)
    if path.suffix != '.bag' or Path(str(path) + '.active').exists():
        raise RuntimeError('Recording must be finalized before viewing: ' + str(path))
    return path


def decode_sample(key, message, project):
    if key in ('local', 'global'):
        p = message.pose.pose.position
        value = (p.x, p.y, yaw_of(message.pose.pose.orientation))
        if not np.isfinite(value).all():
            raise ValueError('non-finite odometry')
    elif key == 'gps':
        if (message.status.status < 0 or not np.isfinite([message.latitude, message.longitude]).all()
                or not -90 <= message.latitude <= 90 or not -180 <= message.longitude <= 180):
            raise ValueError('invalid GPS fix')
        value = project_wgs84(message.latitude, message.longitude, project['origin'])
    elif key == 'speed':
        value = float(message.speed)
        if not math.isfinite(value):
            raise ValueError('non-finite speed')
    elif key in ('state', 'valid'):
        value = message.data
    elif key.startswith('imu_'):
        value = yaw_of(message.orientation)
    elif key == 'calibration':
        statuses = [status for status in message.status if status.name == 'CalibratedIMU']
        if not statuses:
            return None
        status = statuses[-1]
        value = {'diagnostic_level': status.level, 'diagnostic_message': status.message}
        for pair in status.values:
            try:
                value[pair.key] = json.loads(pair.value)
            except ValueError:
                value[pair.key] = pair.value
    elif key == 'diagnostics':
        value = {}
        for status in message.status:
            if status.hardware_id != 'mando_localization':
                continue
            value[status.name] = {
                'level': int(status.level),
                'message': status.message,
                'values': {pair.key: pair.value for pair in status.values},
            }
        if not value:
            return None
    else:
        # LaserScan ranges are already float32 on the wire; keeping compact
        # arrays avoids retaining millions of boxed Python float objects.
        value = {'ranges': np.asarray(message.ranges, dtype=np.float32),
                 'angle_min': message.angle_min, 'angle_increment': message.angle_increment,
                 'range_min': message.range_min, 'range_max': message.range_max}
    return value


class SceneData:
    """Same decoding, anchor, ordering and retention rules for files and live input."""
    def __init__(self, rddf_dir, source_start=None, duration=0., processed_bag=None,
                 limit=200000, frame_mode='first_gps_translation'):
        self.routes, self.project = load_rddf(Path(rddf_dir))
        self.rddf_dir = Path(rddf_dir)
        if frame_mode not in ('first_gps_translation', 'rddf_map', 'odometry'):
            raise ValueError('unknown viewer frame_mode: ' + str(frame_mode))
        self.frame_mode = frame_mode
        if frame_mode == 'odometry':
            self.routes = {}  # Unanchored odometry does not locate the vehicle on RDDF.
        self.start, self.duration, self.limit = source_start, duration, limit
        self.data = {key: [] for key in TOPICS}
        self.times = {key: [] for key in TOPICS}
        self.mount = None
        self.shift = None if frame_mode == 'first_gps_translation' else np.zeros(2)
        self.first_local = self.first_gps = None
        self.summary = {'processed_bag': str(processed_bag) if processed_bag else None,
                        'common_xy_translation_m': self.shift.tolist() if self.shift is not None else None,
                        'frame_mode': frame_mode, 'additional_viewer_yaw_rotation_rad': 0.,
                        'skipped': 0, 'buffer_dropped': 0, 'clock_resets': 0}
        self.last_receipt = None

    def reset(self, stamp):
        for key in TOPICS:
            self.data[key].clear(); self.times[key].clear()
        self.first_local = self.first_gps = None
        self.shift = None if self.frame_mode == 'first_gps_translation' else np.zeros(2)
        self.start, self.duration = stamp, 0.
        self.summary['common_xy_translation_m'] = self.shift.tolist() if self.shift is not None else None
        self.summary['clock_resets'] += 1

    def ingest(self, key, message, stamp, live=False):
        if key == 'tf':
            for tr in message.transforms:
                if tr.header.frame_id.lstrip('/') == 'base_link' and tr.child_frame_id.lstrip('/') == 'laser_link':
                    q, p = tr.transform.rotation, tr.transform.translation
                    vals = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                    if np.isfinite(vals).all() and abs(np.linalg.norm(vals[3:])-1)<.01:
                        self.mount = tr
            return
        if not math.isfinite(stamp) or stamp <= 0:
            self.summary['skipped'] += 1; return
        if self.start is None:
            self.start = stamp
        if live and self.last_receipt is not None and stamp < self.last_receipt-1.0:
            self.reset(stamp)
        self.last_receipt = stamp
        elapsed = stamp-self.start
        if elapsed < 0 or (not live and elapsed > self.duration):
            self.summary['skipped'] += 1; return
        if self.times[key] and elapsed < self.times[key][-1]:
            self.summary['skipped'] += 1; return
        try:
            value = decode_sample(key, message, self.project)
            if value is None: return
        except (ValueError, TypeError, OverflowError):
            self.summary['skipped'] += 1; return
        self.times[key].append(elapsed); self.data[key].append(value)
        if live: self.duration = max(self.duration, elapsed)
        if key == 'local' and self.first_local is None: self.first_local = np.asarray(value[:2])
        if key == 'gps' and self.first_gps is None: self.first_gps = np.asarray(value)
        if self.shift is None and self.first_local is not None and self.first_gps is not None:
            self.shift = self.first_gps-self.first_local
            self.summary['common_xy_translation_m'] = self.shift.tolist()
        limit = min(self.limit, 6000) if key == 'scan' else self.limit
        if live and len(self.data[key]) > limit:
            count = max(1,limit//10)
            del self.data[key][:count]; del self.times[key][:count]
            self.summary['buffer_dropped'] += count

    def points(self, key, count=None):
        width = 2 if key == 'gps' else 3
        values = np.asarray(self.data[key][:count], dtype=float).reshape((-1,width)).copy()
        if key != 'gps':
            if self.shift is None: return np.empty((0,width))
            values[:,:2] += self.shift
        return values

    def view_bounds(self):
        """Fit only observed odometry in the unanchored mode, including a fresh origin."""
        keys = ('local', 'global') if self.frame_mode == 'odometry' else ('local', 'global', 'gps')
        arrays = list(self.routes.values()) + [self.points(key)[:, :2] for key in keys]
        arrays = [points for points in arrays if len(points)]
        points = np.vstack(arrays) if arrays else np.zeros((1, 2))
        lower, upper = points.min(axis=0), points.max(axis=0)
        minimum_span = 20. if self.frame_mode == 'odometry' else 1.
        return (lower+upper)/2, np.maximum(upper-lower, minimum_span)


def load_data(processed_bag, source_bag, rddf_dir, limit, frame_mode='first_gps_translation'):
    processed_bag = final_bag(processed_bag)
    source_bag = final_bag(source_bag) if source_bag else processed_bag
    with rosbag.Bag(str(source_bag)) as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
    model = SceneData(rddf_dir, start, end-start, processed_bag, limit, frame_mode)
    reverse = {value: key for key,value in TOPICS.items()}
    with rosbag.Bag(str(processed_bag)) as bag:
        for topic,message,stamp in bag.read_messages(topics=list(reverse)+['/tf_static']):
            model.ingest('tf' if topic == '/tf_static' else reverse[topic],message,stamp.to_sec())
    if not any(model.data[key] for key in ('local','global')):
        raise ValueError('Selected bag has no computed Local/Global output. Recompute with replay.launch first.')
    model.summary.update(counts={key:len(rows) for key,rows in model.data.items()},duration_s=model.duration,
                         source_bag=str(source_bag))
    return model

def run_gui(model, seek, live=False, config=None):
    import rospy
    from geometry_msgs.msg import Point, TransformStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String, Float64, Bool
    from python_qt_binding import QtCore, QtGui, QtWidgets
    from rviz import bindings as rviz
    from tf.transformations import quaternion_matrix

    class Slider(QtWidgets.QSlider):
        def point(self, event):
            return round(self.maximum() * max(0, min(1, event.x()/max(1, self.width()))))

        def mousePressEvent(self, event):
            if event.button() == QtCore.Qt.LeftButton:
                self.setSliderDown(True)
                self.setValue(self.point(event))

        def mouseMoveEvent(self, event):
            if self.isSliderDown():
                self.setValue(self.point(event))

        def mouseReleaseEvent(self, event):
            self.setSliderDown(False)

    class Viewer(QtWidgets.QWidget):
        seek_signal = QtCore.Signal(float)
        follow_signal = QtCore.Signal(bool)
        initialization_signal = QtCore.Signal(str)

        def __init__(self):
            super().__init__()
            self.model = model
            self.routes, self.data, self.times = model.routes, model.data, model.times
            self.duration, self.mount, self.summary = model.duration, model.mount, model.summary
            self.follow_live = live
            self.inbox = queue.Queue(maxsize=10000)
            self.input_dropped = 0
            self.initialization = {}
            self.selection_active = False
            self.preview = None
            self.candidate_menu = None
            self.scene_keys = set()
            self.selection_message = ''
            from rddf_initialization_core import RddfRouteMap
            self.route_map = None if model.frame_mode == 'odometry' else RddfRouteMap(model.rddf_dir)
            self.current_rddf_max_distance = float(config.get('current_rddf_max_distance_m', 5.0))
            self.current_rddf_ambiguity_distance = float(config.get('current_rddf_ambiguity_distance_m', 0.5))
            if (not math.isfinite(self.current_rddf_max_distance) or self.current_rddf_max_distance <= 0
                    or not math.isfinite(self.current_rddf_ambiguity_distance)
                    or self.current_rddf_ambiguity_distance < 0):
                raise ValueError('invalid current RDDF display distances')
            self.manual_enabled = (live and model.frame_mode == 'rddf_map'
                                   and config.get('manual_initialization_enabled', True))
            self.manual_snap_distance = float(config.get('manual_snap_distance_m', 5.0))
            self.imu_raw_stale_sec = float(config.get('imu_raw_stale_sec', 0.5))
            if not math.isfinite(self.manual_snap_distance) or self.manual_snap_distance <= 0:
                raise ValueError('manual_snap_distance_m must be finite and positive')
            if not math.isfinite(self.imu_raw_stale_sec) or self.imu_raw_stale_sec <= 0:
                raise ValueError('imu_raw_stale_sec must be finite and positive')
            self.position = self.duration if seek is None else max(0.0, min(self.duration, seek))
            self.playing, self.last, self.rate, self.drag_play = False, time.monotonic(), 1.0, False
            rospy.init_node('mando_localization_rviz', disable_signals=True)
            self.scene = rospy.Publisher(PREFIX+'/scene', MarkerArray, queue_size=1, latch=True)
            self.status = rospy.Publisher(PREFIX+'/status', String, queue_size=1, latch=True)
            self.tf = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
            if self.manual_enabled:
                self.manual_request = rospy.Publisher(INITIALIZATION_PREFIX+'/manual_request', String,
                                                      queue_size=1, latch=False)
                self.manual_active = rospy.Publisher(INITIALIZATION_PREFIX+'/manual_active', Bool,
                                                     queue_size=1, latch=True)
                self.manual_active.publish(Bool(False))
                self.initialization_signal.connect(self.initialization_changed)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = FRAME+'_world', FRAME
            transform.transform.rotation.w = 1
            self.tf.publish(TFMessage([transform]))
            self.seek_signal.connect(self.seek)
            self.follow_signal.connect(lambda follow: self.go_latest() if follow else setattr(self, "follow_live", False))
            self.follow_remote = rospy.Subscriber(PREFIX+"/follow_live", Bool, lambda msg: self.follow_signal.emit(msg.data))
            self.remote = rospy.Subscriber(PREFIX+'/seek_seconds', Float64,
                                            lambda msg: self.seek_signal.emit(msg.data))
            self.setWindowTitle('Localization · RDDF 공통 RViz · '+('실시간 입력' if live else str(model.summary['processed_bag'])))
            layout = QtWidgets.QVBoxLayout(self)
            heading = QtWidgets.QLabel('RDDF 공통 뷰어  |  초록 Local · 빨강 Global · 보라 Raw GPS · 파랑 RDDF')
            heading.setStyleSheet('font-size:16px;font-weight:bold;padding:6px')
            layout.addWidget(heading)
            self.current_rddf_label = QtWidgets.QLabel(
                'IMU·엔코더 Odometry · GPS/RDDF 위치 정합 없음'
                if model.frame_mode == 'odometry' else 'RDDF: waiting for Global position')
            self.current_rddf_label.setObjectName('current_rddf_label')
            self.current_rddf_label.setWordWrap(True)
            self.current_rddf_label.setStyleSheet('font-size:16px;font-weight:bold;padding:6px')
            layout.addWidget(self.current_rddf_label)
            if self.manual_enabled:
                selection = QtWidgets.QHBoxLayout()
                self.select_button = QtWidgets.QPushButton('시작 위치 선택')
                self.select_button.setObjectName('initialization_select_button')
                self.select_button.setCheckable(True)
                self.select_button.setEnabled(False)
                self.select_button.setMinimumHeight(37)
                self.select_button.toggled.connect(self.set_selection_active)
                selection.addWidget(self.select_button)
                selection.addWidget(QtWidgets.QLabel('진행 경로'))
                self.route_choice = QtWidgets.QComboBox()
                self.route_choice.setObjectName('initialization_route_choice')
                self.route_choice.addItem('자동 (가까운 RDDF)', '')
                for name in sorted(self.route_map.routes):
                    self.route_choice.addItem(name, name)
                self.route_choice.currentIndexChanged.connect(self.route_changed)
                self.route_choice.setEnabled(False)
                selection.addWidget(self.route_choice)
                self.initialization_label = QtWidgets.QLabel('초기화 노드 상태 대기')
                self.initialization_label.setWordWrap(True)
                # Hover text must not resize the Ogre viewport under the pointer.
                self.initialization_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                                                        QtWidgets.QSizePolicy.Fixed)
                self.initialization_label.setFixedHeight(
                    self.initialization_label.fontMetrics().lineSpacing() * 5 + 8)
                selection.addWidget(self.initialization_label, 1)
                layout.addLayout(selection)
            self.frame = rviz.VisualizationFrame()
            self.frame.setSplashPath('')
            self.frame.initialize()
            # RViz retains panel-menu QAction pointers until frame destruction.
            # Hiding bars preserves those owners and avoids shutdown use-after-free.
            self.frame.menuBar().hide()
            self.frame.statusBar().hide()
            map_row = QtWidgets.QHBoxLayout()
            map_row.setSpacing(8)
            self.sensor_panel = QtWidgets.QFrame()
            self.sensor_panel.setObjectName('sensor_status_panel')
            self.sensor_panel.setMinimumWidth(245)
            self.sensor_panel.setMaximumWidth(285)
            self.sensor_panel.setStyleSheet(
                'QFrame#sensor_status_panel{background:#20242a;border:1px solid #454b54;'
                'border-radius:6px;} QLabel{color:#e8eaed;}')
            sensor_layout = QtWidgets.QVBoxLayout(self.sensor_panel)
            sensor_title = QtWidgets.QLabel('센서 및 추정 상태')
            sensor_title.setStyleSheet('font-size:16px;font-weight:bold;padding:8px 5px')
            sensor_layout.addWidget(sensor_title)
            self.sensor_labels = {}
            for key, title in (
                    ('IMU', 'IMU 센서'), ('IMU_CALIBRATED', 'IMU 보정 출력'),
                    ('ENCODER', '엔코더'), ('GPS_POSE', 'GPS'),
                    ('LOCAL_ODOMETRY', 'Local EKF'),
                    ('GLOBAL_ODOMETRY', 'Global EKF')):
                label = QtWidgets.QLabel('●  {}\n    진단 수신 대기'.format(title))
                label.setObjectName('sensor_status_'+key.lower())
                label.setWordWrap(True)
                label.setMinimumHeight(58)
                label.setStyleSheet(
                    'color:#9aa0a6;background:#292e35;border-radius:4px;padding:7px;')
                sensor_layout.addWidget(label)
                self.sensor_labels[key] = (title, label)
            sensor_layout.addStretch(1)
            self.sensor_overall = QtWidgets.QLabel('전체 Localization\n진단 수신 대기')
            self.sensor_overall.setWordWrap(True)
            self.sensor_overall.setStyleSheet(
                'font-weight:bold;color:#9aa0a6;background:#292e35;'
                'border-radius:4px;padding:9px;')
            sensor_layout.addWidget(self.sensor_overall)
            map_row.addWidget(self.sensor_panel)
            map_row.addWidget(self.frame, 1)
            layout.addLayout(map_row, 1)
            manager = self.frame.getManager()
            # Startup position goes through the explicit RDDF admission path.
            # Default RViz pose/goal tools must not publish during bag browsing.
            tools = manager.getToolManager()
            for index in reversed(range(tools.numTools())):
                if tools.getTool(index).getClassId() in ('rviz/SetInitialPose', 'rviz/SetGoal', 'rviz/PublishPoint'):
                    tools.removeTool(index)
            manager.setFixedFrame(FRAME)
            manager.removeAllDisplays()
            display = manager.createDisplay('rviz/MarkerArray', '기록 시점 장면', True)
            display.subProp('Marker Topic').setValue(PREFIX+'/scene')
            grid = manager.createDisplay('rviz/Grid', '5 m 격자', True)
            grid.subProp('Cell Size').setValue(5.0)
            grid.subProp('Plane Cell Count').setValue(150)
            view = manager.getViewManager()
            view.setCurrentViewControllerType('rviz/TopDownOrtho')
            self.view = view.getCurrent()
            self.view.subProp('Angle').setValue(0.0)
            # RenderPanel is not exported in Noetic's Python SIP bindings. Its
            # QWidget wrapper still exposes the Qt metaobject and mouse events.
            panels = [widget for widget in self.frame.centralWidget().findChildren(QtWidgets.QWidget)
                      if widget.metaObject().className() == 'rviz::RenderPanel']
            if len(panels) != 1:
                raise RuntimeError('cannot identify RViz RenderPanel for map coordinates')
            self.render_panel = panels[0]
            self.render_panel.setObjectName('initialization_rviz_render_panel')
            if self.manual_enabled:
                self.render_panel.setMouseTracking(True)
                self.render_panel.installEventFilter(self)
            self.info = QtWidgets.QLabel()
            self.info.setStyleSheet('font-size:14px;padding:5px')
            self.info.setWordWrap(True)
            layout.addWidget(self.info)
            self.slider = Slider(QtCore.Qt.Horizontal)
            self.slider.setRange(0, math.ceil(self.duration*10))
            self.slider.setMinimumHeight(32)
            layout.addWidget(self.slider)
            self.slider.sliderPressed.connect(self.drag_start)
            self.slider.sliderReleased.connect(self.drag_end)
            self.slider.valueChanged.connect(lambda value: self.seek(value/10) if self.slider.isSliderDown() else None)
            controls = QtWidgets.QHBoxLayout()
            layout.addLayout(controls)

            def button(label, callback):
                widget = QtWidgets.QPushButton(label)
                widget.setMinimumHeight(37)
                widget.clicked.connect(callback)
                controls.addWidget(widget)
                return widget

            button('처음', lambda: self.seek(0))
            button('−10초', lambda: self.seek(self.position-10))
            self.play_button = button('▶ 재생', self.toggle)
            button('+10초', lambda: self.seek(self.position+10))
            button('최신 / 끝', self.go_latest)
            button('전체 경로 맞춤', self.fit_view)
            speed = QtWidgets.QComboBox()
            speed.addItems(['0.25×', '0.5×', '1×', '2×', '4×', '8×'])
            speed.setCurrentIndex(2)
            speed.currentIndexChanged.connect(lambda index: setattr(self, 'rate', [.25, .5, 1, 2, 4, 8][index]))
            controls.addWidget(speed)
            self.jump = QtWidgets.QLineEdit()
            self.jump.setPlaceholderText('분:초 (예: 08:50)')
            self.jump.setMaximumWidth(180)
            self.jump.returnPressed.connect(self.jump_time)
            controls.addWidget(self.jump)
            button('이동', self.jump_time)
            self.clock_label = QtWidgets.QLabel()
            controls.addWidget(self.clock_label)
            frame_note = {
                'rddf_map': 'RDDF map 좌표 그대로 표시',
                'odometry': '노드 Odometry 위치·yaw 그대로 표시 · 지도 위치 정합 없음',
                'first_gps_translation': 'Local/Global 동일 XY 이동 · yaw 회전 없음',
            }[model.frame_mode]
            note = QtWidgets.QLabel('Space 재생/정지 · ←/→ 10초 · '+frame_note+
                                   ' · 시간 탐색은 화면만 이동'+
                                   (' · RDDF는 설계 경로' if model.frame_mode != 'odometry' else ''))
            note.setStyleSheet('color:#777;padding:4px')
            note.setWordWrap(True)
            layout.addWidget(note)
            for key, callback in [('Space', self.toggle), ('Left', lambda: self.seek(self.position-10)),
                                  ('Right', lambda: self.seek(self.position+10))]:
                QtWidgets.QShortcut(QtGui.QKeySequence(key), self, callback)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(int(config['refresh_ms']))
            self.subscribers = self.subscribe_live() if live else []
            if self.manual_enabled:
                self.subscribers.append(rospy.Subscriber(INITIALIZATION_PREFIX+'/status', String,
                    lambda message: self.initialization_signal.emit(message.data), queue_size=10))
            self.resize(1600, 1050)
            self.fit_view()
            self.render()

        def initialization_changed(self, payload):
            try:
                status = json.loads(payload)
                if not isinstance(status, dict) or not isinstance(status.get('state'), str):
                    raise ValueError('initialization status requires state')
            except (ValueError, TypeError):
                self.selection_message = '초기화 상태 메시지가 올바르지 않습니다'
                return
            if status == self.initialization:
                return
            self.initialization = status
            locked = status['state'] in ('READY', 'INITIALIZING', 'FAULT')
            self.select_button.setEnabled(not locked)
            self.route_choice.setEnabled(not locked and self.selection_active)
            if locked and self.selection_active:
                self.select_button.setChecked(False)
            # The refresh timer renders after draining live inputs. Rendering
            # here as well can queue stale scenes ahead of current odometry.

        def set_selection_active(self, active):
            if active and self.initialization.get('state') in ('READY', 'INITIALIZING', 'FAULT'):
                self.select_button.setChecked(False)
                return
            self.selection_active = bool(active)
            if self.candidate_menu is not None:
                self.candidate_menu.close()
            self.manual_active.publish(Bool(self.selection_active))
            self.preview = None
            self.selection_message = ''
            self.route_choice.setEnabled(self.selection_active)
            self.select_button.setText('선택 취소 · GPS 대기' if active else '시작 위치 선택')
            self.render_panel.setCursor(QtCore.Qt.CrossCursor if active else QtCore.Qt.ArrowCursor)
            if active:
                self.go_latest()
                self.playing = False
                self.frame.getManager().setFixedFrame(FRAME)
                self.frame.getManager().getViewManager().setCurrentViewControllerType('rviz/TopDownOrtho')
                self.view = self.frame.getManager().getViewManager().getCurrent()
                self.view.subProp('Angle').setValue(0.)
                self.view.subProp('Target Frame').setValue('<Fixed Frame>')
            self.render()

        def route_changed(self, _index):
            if self.candidate_menu is not None:
                self.candidate_menu.close()
            self.preview = None
            self.selection_message = ''
            self.render()

        def update_preview(self, point):
            route = self.route_choice.currentData() or None
            try:
                view = self.frame.getManager().getViewManager().getCurrent()
                if view.getClassId() != 'rviz/TopDownOrtho':
                    raise ValueError('시작 위치 선택은 TopDownOrtho 화면에서만 가능합니다')
                if (self.frame.getManager().getFixedFrame() != FRAME
                        or str(view.subProp('Target Frame').getValue()) not in ('<Fixed Frame>', FRAME)):
                    raise ValueError('RDDF 기준 프레임이 변경됐습니다. 시작 위치 선택을 다시 켜세요')
                props = [float(view.subProp(name).getValue()) for name in ('Scale', 'X', 'Y', 'Angle')]
                x, y = topdown_screen_to_map(point.x(), point.y(), self.render_panel.width(),
                    self.render_panel.height(), *props, pixel_ratio=self.render_panel.devicePixelRatioF())
                self.preview = self.route_map.match(x, y, self.manual_snap_distance, route_name=route)
                if self.preview.get('accepted'):
                    self.selection_message = ''
                elif self.preview.get('reason') == 'AMBIGUOUS_ROUTE':
                    self.selection_message = '경로가 겹칩니다. 클릭해서 경로와 차량 방향을 고르세요'
                else:
                    self.selection_message = '선택한 RDDF 선 가까이 마우스를 이동하세요: '+self.preview.get('reason', '')
            except (ValueError, TypeError, RuntimeError) as error:
                self.preview = None
                self.selection_message = str(error)

        def eventFilter(self, watched, event):
            if watched is getattr(self, 'render_panel', None) and self.selection_active:
                if self.candidate_menu is not None:
                    return super().eventFilter(watched, event)
                kind = event.type()
                if kind == QtCore.QEvent.Leave:
                    self.preview = None
                elif kind in (QtCore.QEvent.MouseMove, QtCore.QEvent.MouseButtonPress,
                              QtCore.QEvent.MouseButtonRelease, QtCore.QEvent.MouseButtonDblClick):
                    self.update_preview(event.pos())
                    if kind == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
                        if self.preview and self.preview.get('reason') == 'AMBIGUOUS_ROUTE':
                            self.choose_overlap(event.globalPos())
                        else:
                            self.submit_manual_pose()
                        self.render()
                        return True
                    if event.buttons() & QtCore.Qt.LeftButton or (
                            kind != QtCore.QEvent.MouseMove and event.button() == QtCore.Qt.LeftButton):
                        return True
                    # The refresh timer draws hover previews at a bounded rate.
            return super().eventFilter(watched, event)

        def choose_overlap(self, position):
            menu = QtWidgets.QMenu(self)
            menu.setObjectName('initialization_overlap_menu')
            menu.addSection('실제 경로 / 차량 앞쪽 방향 선택')

            def preview_candidate(candidate):
                self.preview = self.route_map.match(candidate['x'], candidate['y'],
                    self.manual_snap_distance, route_name=candidate['route'],
                    segment_index=candidate['index'])
                self.render()

            def submit_candidate(candidate):
                preview_candidate(candidate)
                self.submit_manual_pose()
                self.render()

            for candidate in self.preview.get('candidates', []):
                action = menu.addAction('{} · 구간 {} · 차량 yaw {:.1f}°'.format(
                    candidate['route'], candidate['index'], math.degrees(candidate['yaw'])))
                action.hovered.connect(lambda candidate=candidate: preview_candidate(candidate))
                action.triggered.connect(lambda checked=False, candidate=candidate: submit_candidate(candidate))

            def closed():
                self.candidate_menu = None
                self.preview = None
                menu.deleteLater()

            menu.aboutToHide.connect(closed)
            self.candidate_menu = menu
            menu.popup(position)

        def submit_manual_pose(self):
            if (not self.selection_active or self.initialization.get('state') in
                    ('READY', 'INITIALIZING', 'FAULT')):
                return
            if not self.preview or not self.preview.get('accepted'):
                self.selection_message = 'RDDF 선 가까이를 클릭하세요. 겹친 지점은 클릭 후 후보를 고르세요'
                return
            if self.manual_request.get_num_connections() < 1:
                self.selection_message = '초기화 노드 연결 대기: 아직 시작 위치를 보내지 않았습니다'
                return
            try:
                payload = manual_initialization_request(self.preview, rospy.Time.now().to_sec())
            except ValueError as error:
                self.selection_message = str(error)
                return
            self.manual_request.publish(String(json.dumps(payload, allow_nan=False)))
            self.selection_message = '선택 위치 전송 완료 · 정지/IMU 확인 및 초기화 결과 대기'

        def fit_view(self):
            center, span = self.model.view_bounds()
            self.view.subProp('X').setValue(float(center[0]))
            self.view.subProp('Y').setValue(float(center[1]))
            scale = .8*self.render_panel.devicePixelRatioF()*min(
                max(600, self.render_panel.width())/span[0], max(500, self.render_panel.height())/span[1])
            self.view.subProp('Scale').setValue(float(scale))

        def enqueue(self, key, message):
            try:
                self.inbox.put_nowait((key,message,rospy.Time.now().to_sec()))
            except queue.Full:
                self.input_dropped += 1

        def subscribe_live(self):
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu, NavSatFix, LaserScan
            from diagnostic_msgs.msg import DiagnosticArray
            from std_msgs.msg import Bool
            from erp42_msgs.msg import SerialFeedBack
            types = dict(local=Odometry, **{'global':Odometry}, gps=NavSatFix, scan=LaserScan,
                         speed=SerialFeedBack, state=String, valid=Bool, imu_raw=Imu,
                         imu_normalized=Imu, imu_calibrated=Imu, calibration=DiagnosticArray,
                         diagnostics=DiagnosticArray)
            subs = [rospy.Subscriber(topic,types[key],lambda msg,k=key:self.enqueue(k,msg),queue_size=100)
                    for key,topic in TOPICS.items()]
            subs.append(rospy.Subscriber('/tf_static',TFMessage,lambda msg:self.enqueue('tf',msg),queue_size=20))
            return subs

        def go_latest(self):
            self.seek(self.duration)
            self.follow_live = live

        def drag_start(self):
            self.drag_play, self.playing = self.playing, False

        def drag_end(self):
            self.playing, self.last = self.drag_play, time.monotonic()

        def seek(self, value):
            if math.isfinite(value):
                self.follow_live = False
                self.position = max(0.0, min(self.duration, value))
                self.last = time.monotonic()
                self.render()

        def toggle(self):
            if self.follow_live:
                self.follow_live = False
                self.playing = False
                return
            if self.position >= self.duration:
                self.position = 0
            self.playing = not self.playing
            self.last = time.monotonic()
            self.render()

        def jump_time(self):
            try:
                value = sum(float(part)*60**index for index, part in enumerate(reversed(self.jump.text().split(':'))))
                if not math.isfinite(value):
                    raise ValueError('non-finite time')
                self.seek(value)
            except ValueError:
                self.jump.setText('분:초 형식으로 입력')

        def tick(self):
            now = time.monotonic()
            if rospy.is_shutdown():
                self.close(); return
            if live:
                for _ in range(10000):
                    try: key,message,stamp = self.inbox.get_nowait()
                    except queue.Empty: break
                    self.model.ingest(key,message,stamp,live=True)
                self.duration, self.mount = self.model.duration, self.model.mount
                self.slider.setMaximum(math.ceil(self.duration*10))
                if self.follow_live: self.position = self.duration
                self.render()
            if self.playing:
                self.position = min(self.duration, self.position+(now-self.last)*self.rate)
                if self.position >= self.duration:
                    self.playing = False
                self.render()
            self.last = now

        def latest(self, key, default=None):
            index = bisect.bisect_right(self.times[key], self.position)-1
            return self.data[key][index] if index >= 0 else default

        def marker(self, ident, kind, color, points, width=.22, namespace='localization_debug'):
            message = Marker()
            message.header.frame_id, message.ns, message.id = FRAME, namespace, ident
            message.type, message.action = kind, Marker.ADD
            message.pose.orientation.w = 1
            message.scale.x = message.scale.y = message.scale.z = width
            message.color.r, message.color.g, message.color.b = color
            message.color.a = 1
            message.points = [Point(float(point[0]), float(point[1]), float(point[2]) if len(point)>2 else 0.0) for point in points]
            return message

        def render_sensor_status(self):
            index = bisect.bisect_right(self.times['diagnostics'], self.position)-1
            diagnostics = self.data['diagnostics'][index] if index >= 0 else {}
            stale = index < 0 or self.position-self.times['diagnostics'][index] > 1.0
            colors = {0: ('#34a853', '#17351f'), 1: ('#fbbc04', '#3b3215'),
                      2: ('#ea4335', '#3c1e1d'), 3: ('#9aa0a6', '#292e35')}
            translations = {
                'OK': '정상', 'DISABLED': '비활성', 'NOT_RECEIVED': '미수신',
                'CONNECTED': '연결됨', 'NOT_CONNECTED': '연결 안 됨',
                'RUNNING': '실행 중', 'NOT_RUNNING': '실행 안 됨',
            }
            for key, (title, label) in self.sensor_labels.items():
                diagnostic_key = 'IMU' if key == 'IMU_CALIBRATED' else key
                status = diagnostics.get(diagnostic_key)
                if key == 'IMU':
                    level, detail = stream_reception_status(
                        self.times['imu_raw'], self.position, self.imu_raw_stale_sec,
                        '미수신' if live else '기록 없음')
                elif stale or status is None:
                    level, detail = 3, '진단 수신 대기' if index < 0 else '진단 끊김'
                else:
                    level = status['level'] if status['level'] in colors else 3
                    detail = translations.get(status['message'], status['message'])
                    if key == 'IMU_CALIBRATED' and status['message'] == 'NOT_RECEIVED':
                        calibration = self.latest('calibration', {})
                        detail = ('RDDF 위치 선택 대기'
                                  if calibration.get('reason') == 'WAITING_FOR_RDDF_POSITION'
                                  else '초기화/보정 대기')
                    if status['message'] == 'DISABLED':
                        level = 3
                foreground, background = colors[level]
                label.setText('●  {}\n    {}'.format(title, detail))
                label.setStyleSheet('color:{};background:{};border-radius:4px;padding:7px;'.format(
                    foreground, background))
            overall = diagnostics.get('LOCALIZATION')
            if stale or overall is None:
                level, detail = 3, '진단 수신 대기' if index < 0 else '진단 끊김'
            else:
                level = overall['level'] if overall['level'] in colors else 3
                detail = translations.get(overall['message'], overall['message'])
            foreground, background = colors[level]
            self.sensor_overall.setText('전체 Localization\n{}'.format(detail))
            self.sensor_overall.setStyleSheet(
                'font-weight:bold;color:{};background:{};border-radius:4px;padding:9px;'.format(
                    foreground, background))

        def render(self):
            markers, current, last_times, counts = [], {}, {}, {}
            selected_route = self.route_choice.currentData() if self.manual_enabled and self.selection_active else None
            if not selected_route and self.selection_active and self.preview and self.preview.get('accepted'):
                selected_route = self.preview['route']
            for name, route in self.routes.items():
                color = (1., .8, .15) if name == selected_route else COLORS['rddf']
                markers.append(self.marker(0, Marker.LINE_STRIP, color, route, .15, 'rddf/'+name))
                markers[-1].pose.position.z = .04 if name == selected_route else .02
            if self.selection_active and self.preview and self.preview.get('accepted'):
                x, y, yaw = (self.preview[key] for key in ('x', 'y', 'yaw'))
                c, s = math.cos(yaw), math.sin(yaw)
                box = np.array([[-.675, -.425], [.675, -.425], [.675, .425], [-.675, .425], [-.675, -.425]])
                points = box@np.array([[c, s], [-s, c]])+[x, y]
                markers.append(self.marker(0, Marker.LINE_STRIP, (1., .8, .15), points, .25,
                                           'initialization_preview'))
                markers[-1].pose.position.z = .15
                markers.append(self.marker(1, Marker.ARROW, (1., .8, .15),
                    [(x, y, .15), (x+3*c, y+3*s, .15)], .5, 'initialization_preview'))
            for key in ('local', 'global', 'gps'):
                count = bisect.bisect_right(self.times[key], self.position)
                counts[key] = count
                if not count:
                    continue
                values, stamps = self.model.points(key,count), np.asarray(self.times[key][:count])
                if not len(values): continue
                last_times[key] = float(stamps[-1])
                boundaries = np.r_[0, np.flatnonzero(np.diff(stamps)>.5)+1, count]
                lines = []
                for left, right in zip(boundaries[:-1], boundaries[1:]):
                    xy = values[left:right, :2]
                    xy = xy[::max(1, math.ceil(len(xy)/5000))]
                    if len(xy)>1:
                        lines.extend(np.stack([xy[:-1], xy[1:]], axis=1).reshape((-1, 2)))
                markers.append(self.marker(0, Marker.LINE_LIST, COLORS[key], lines, .3, 'track/'+key))
                if self.position-stamps[-1]>.5:
                    continue
                current[key] = values[-1]
                if key == 'gps':
                    markers.append(self.marker(1, Marker.POINTS, COLORS[key], [values[-1]], .9, 'track/'+key))
                else:
                    x, y, yaw = values[-1]
                    box = np.array([[-.675, -.425], [.675, -.425], [.675, .425], [-.675, .425], [-.675, -.425]])
                    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                    markers.append(self.marker(1, Marker.LINE_STRIP, COLORS[key], box@rotation.T+[x, y], .18, 'track/'+key))
                    markers.append(self.marker(2, Marker.ARROW, COLORS[key], [(x, y), (x+3*math.cos(yaw), y+3*math.sin(yaw))], .4, 'track/'+key))
            # Require a recent validity sample as well as a recent Global pose.
            # Following live time also expires markers if every input stops.
            display_now = self.position
            if live and self.follow_live and self.model.start is not None:
                display_now = max(display_now, rospy.Time.now().to_sec()-self.model.start)
            global_position = current.get('global')
            if display_now-last_times.get('global', -math.inf) > .5:
                global_position = None
            valid_index = bisect.bisect_right(self.times['valid'], self.position)-1
            match_valid = (valid_index >= 0 and bool(self.data['valid'][valid_index])
                           and display_now-self.times['valid'][valid_index] <= 1.0)
            if self.model.frame_mode == 'odometry':
                rddf_match = dict(accepted=False, reason='RDDF_DISABLED', routes=[],
                                  text='IMU·엔코더 Odometry · GPS/RDDF 위치 정합 없음')
            else:
                rddf_match = current_rddf_match(self.route_map, global_position, match_valid,
                    self.current_rddf_max_distance, self.current_rddf_ambiguity_distance)
            route_color = (1., .8, .15) if rddf_match['accepted'] else (1., .5, .15)
            if rddf_match['accepted']:
                names = [rddf_match['route']]
            elif rddf_match['reason'] == 'AMBIGUOUS_ROUTE':
                names = rddf_match['routes']
            else:
                names = []
            for name in current_rddf_members(self.route_map, names) if self.route_map is not None else []:
                markers.append(self.marker(0, Marker.LINE_STRIP, route_color,
                    self.routes[name], .35, 'current_rddf/'+name))
                markers[-1].pose.position.z = .08
            route_text = rddf_match['text']
            if self.model.frame_mode == 'first_gps_translation':
                route_text += ' (display coordinates)'
            self.current_rddf_label.setText(route_text)
            if global_position is not None and self.model.frame_mode != 'odometry':
                label = self.marker(0, Marker.TEXT_VIEW_FACING, route_color, [], 1.2, 'current_rddf_label')
                label.pose.position.x = float(global_position[0])
                label.pose.position.y = float(global_position[1])+2.5
                label.pose.position.z = 1.5
                label.text = route_text
                markers.append(label)
            scan_index = bisect.bisect_right(self.times['scan'], self.position)-1
            scan_fresh = scan_index >= 0 and self.position-self.times['scan'][scan_index]<.5
            scan_visible = scan_fresh and 'global' in current and self.mount is not None
            if scan_visible:
                scan = self.data['scan'][scan_index]
                q, tr = self.mount.transform.rotation, self.mount.transform.translation
                rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
                ranges = scan['ranges']
                angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
                valid_ranges = np.isfinite(ranges)&(ranges>=scan['range_min'])&(ranges<=scan['range_max'])
                xyz = np.column_stack([ranges[valid_ranges]*np.cos(angles[valid_ranges]), ranges[valid_ranges]*np.sin(angles[valid_ranges]), np.zeros(sum(valid_ranges))])@rotation.T
                xyz += [tr.x, tr.y, tr.z]
                x, y, yaw = current['global']
                c, s = math.cos(yaw), math.sin(yaw)
                xyz[:, :2] = xyz[:, :2]@np.array([[c, s], [-s, c]])+[x, y]
                markers.append(self.marker(0, Marker.POINTS, (.1, 1, 1), xyz, .10, 'scan'))
            updates, self.scene_keys = scene_marker_updates(markers, self.scene_keys)
            self.scene.publish(MarkerArray(updates))
            state, valid = self.latest('state', '기록 없음'), self.latest('valid', False)
            speed = self.latest('speed')
            speed_text = '기록 없음' if speed is None else '{:.2f} m/s'.format(speed)
            calibration = self.latest('calibration', {})
            calib_text = 'CalibratedIMU 진단: 기록 없음'
            if calibration:
                calibration_state = calibration.get('state', '알 수 없음')
                calibration_state = {
                    'RDDF_INITIALIZED': 'RDDF 초기 정렬 · GNSS 미보정',
                    'ALIGNING': 'GNSS 보정 적용 중',
                    'CALIBRATED': 'GNSS 보정 적용 완료',
                }.get(calibration_state, calibration_state)
                calib_text = 'CalibratedIMU {} | correction_count={} | 목표 offset={}° / 적용={}° | {}'.format(
                    calibration_state, calibration.get('correction_count', '?'),
                    calibration.get('yaw_offset_deg', '?'), calibration.get('applied_yaw_offset_deg', '?'),
                    calibration.get('diagnostic_message', ''))
            headings = heading_readout(current, self.latest('imu_raw'), self.latest('imu_calibrated'))
            self.info.setText('상태: {}  valid={}  속도 {}  LiDAR: {}\n{}\n{}'.format(
                state + (' | GPS·Local 기준점 대기' if self.model.shift is None else ''), valid, speed_text, '기록 있음' if scan_visible else '이 시점 표시 없음', calib_text, headings))
            self.render_sensor_status()
            if self.manual_enabled:
                status = self.initialization
                state_text = {
                    'WAITING_FOR_GPS': 'GPS 대기 · GPS가 없으면 시작 위치 선택',
                    'WAITING_FOR_MANUAL': '지도에서 시작 위치 선택 대기',
                    'WAITING_FOR_STATIONARY': '차량 정지 확인 대기',
                    'WAITING_FOR_IMU': 'IMU 입력 대기',
                    'INITIALIZING': '초기 위치·방향 적용 중',
                    'READY': '초기 피팅 완료 · 시작 위치 선택 잠금',
                    'FAULT': '초기화 오류',
                }.get(status.get('state'), '초기화 노드 상태 대기')
                text = state_text
                if status.get('route'):
                    text += ' | {} / 구간 {}'.format(status['route'], status.get('index', '?'))
                if status.get('reason'):
                    text += ' | '+str(status['reason'])
                if self.selection_active:
                    text += '\n차량을 선택 경로 중앙에 진행 방향으로 놓고 클릭 · Esc 선택 취소'
                    if self.preview and self.preview.get('accepted'):
                        text += ' | {}: ({:.2f}, {:.2f}) m, yaw {:.2f}°'.format(
                            self.preview['route'], self.preview['x'], self.preview['y'],
                            math.degrees(self.preview['yaw']))
                if self.selection_message:
                    text += '\n'+self.selection_message
                self.initialization_label.setText(text)
                self.initialization_label.setToolTip(text)
            self.clock_label.setText('{:02d}:{:04.1f} / {:02d}:{:04.1f}'.format(
                int(self.position)//60, self.position%60, int(self.duration)//60, self.duration%60))
            self.play_button.setText('⏸ 화면 정지' if self.playing or self.follow_live else '▶ 화면 재생')
            if not self.slider.isSliderDown():
                self.slider.blockSignals(True)
                self.slider.setValue(round(self.position*10))
                self.slider.blockSignals(False)
            self.status.publish(String(json.dumps({
                'mode': 'live' if live else 'recorded', 'following_live': self.follow_live, 'input_dropped': self.input_dropped, 'buffer_dropped': self.summary['buffer_dropped'], 'clock_resets': self.summary['clock_resets'], 'anchor_ready': self.model.shift is not None, 'elapsed': self.position, 'duration': self.duration, 'playing': self.playing,
                'counts': counts, 'last_sample_times': last_times, 'scan_visible': bool(scan_visible),
                'state': state, 'valid': bool(valid), 'calibration': calibration,
                'common_xy_translation_m': self.summary['common_xy_translation_m'],
                'frame_mode': self.model.frame_mode,
                'current_rddf': rddf_match,
                'initialization': self.initialization,
                'manual_selection_active': self.selection_active,
                'manual_preview': self.preview,
                'processed_bag': self.summary['processed_bag'],
            }, allow_nan=False)))

        def closeEvent(self, event):
            self.timer.stop()
            self.frame.getManager().stopUpdate()
            if self.manual_enabled:
                self.manual_active.publish(Bool(False))
            rospy.signal_shutdown('current-code viewer closed')
            event.accept()

        def keyPressEvent(self, event):
            if event.key() == QtCore.Qt.Key_Escape and self.selection_active:
                self.select_button.setChecked(False)
                event.accept()
                return
            super().keyPressEvent(event)

    app = QtWidgets.QApplication(sys.argv)
    window = Viewer()
    window.showMaximized()
    QtCore.QTimer.singleShot(500, window.fit_view)
    window.raise_()
    window.activateWindow()
    print('READY unified RViz', 'live' if live else model.summary['processed_bag'], flush=True)
    result = app.exec_()
    # RViz panel destruction posts Qt events. Destroy the frame while the Qt
    # application still exists, including when Python signal cycles retain it.
    window.deleteLater()
    app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    return result


def main():
    import rospy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=PACKAGE/'config/localization_viewer.yaml')
    parser.add_argument('--mode', choices=['live','recorded'], default='live')
    parser.add_argument('--processed-bag', type=Path)
    parser.add_argument('--source-bag', type=Path)
    parser.add_argument('--rddf-dir', type=Path)
    parser.add_argument('--seek', type=float)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        config = yaml.safe_load(args.config.read_text())
        TOPICS.update(config['topics'])
        COLORS.update({key:tuple(value) for key,value in config['colors'].items()})
        if args.seek is not None and not math.isfinite(args.seek): raise ValueError('seek must be finite')
        rddf = args.rddf_dir or Path(config['rddf_directory'])
        if not rddf.is_absolute(): rddf = PACKAGE/rddf
        if config['max_samples_per_topic']<2 or config['refresh_ms']<20: raise ValueError('invalid buffer/refresh limits')
        if args.mode == 'recorded':
            if not args.processed_bag: raise ValueError('--processed-bag is required; no historical bag is selected automatically')
            model = load_data(args.processed_bag,args.source_bag,rddf,config['max_samples_per_topic'],
                              frame_mode=config.get('frame_mode', 'first_gps_translation'))
        else:
            if args.processed_bag or args.source_bag: raise ValueError('bag arguments require --mode recorded')
            default_frame_mode = 'first_gps_translation'
            if 'frame_mode' not in config:
                try:
                    if rospy.get_param('/mando_localization/initialization/enabled', False):
                        default_frame_mode = 'rddf_map'
                except (OSError, rospy.ROSException):
                    if not args.check_only:
                        raise
            model = SceneData(rddf,limit=config['max_samples_per_topic'],
                              frame_mode=config.get('frame_mode', default_frame_mode))
        if args.check_only:
            print(json.dumps(model.summary,indent=2,ensure_ascii=False)); return 0
        return run_gui(model,args.seek,args.mode=='live',config)
    except (OSError,ValueError,RuntimeError,rosbag.ROSBagException) as error:
        print('ERROR:',error,file=sys.stderr); return 1


if __name__ == '__main__':
    sys.exit(main())
