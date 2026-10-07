#!/usr/bin/env python3
"""Bench-only keyboard encoder feedback; orientation always comes from live IMU."""
import json
import math
import signal
import sys
import threading
import time

import rospy
from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
from python_qt_binding import QtCore, QtWidgets
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class KeyboardEncoder(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('IMU + Encoder Test | Hold W')
        self.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.resize(470, 390)
        self.move(95, 170)
        self.speed = float(rospy.get_param('~speed_mps', 1.0))
        self.tick_m = float(rospy.get_param('~simulated_meter_per_tick', 0.01))
        if not (math.isfinite(self.speed) and 0 < self.speed <= 5
                and math.isfinite(self.tick_m) and self.tick_m > 0):
            raise ValueError('Invalid bench speed or simulated tick size')
        self.delta = max(1, round(self.speed * .1 / self.tick_m))
        if self.delta > 100000:
            raise ValueError('simulated encoder delta exceeds production limit')
        self.held = False
        self.alive = 0
        self.distance = 0.
        self.previous_time = time.monotonic()
        self.previous_speed = 0.
        self.lock = threading.Lock()
        self.imu_receipt = None
        self.imu_stamp = None
        self.values = {}
        topic = rospy.get_param('/mando_localization/interfaces/topics/encoder_state')
        self.publisher = rospy.Publisher(topic, SerialFeedBack, queue_size=1)
        self.status = rospy.Publisher('/mando_localization/test/keyboard_status', String, queue_size=1)
        self.subscribers = [
            rospy.Subscriber('/mando_localization/internal/driver/imu', Imu, self.imu, queue_size=1),
            rospy.Subscriber('/molit/localization/imu/calibrated', Imu,
                             lambda m: self.store('calibrated_yaw', yaw(m.orientation)), queue_size=1),
        ]
        for name in ('local', 'global'):
            self.subscribers.append(rospy.Subscriber(
                '/molit/localization/' + name + '/odometry', Odometry,
                lambda m, n=name: self.store(n, (m.pose.pose.position.x,
                    m.pose.pose.position.y, yaw(m.pose.pose.orientation))), queue_size=1))
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel('실제 IMU + 가상 엔코더')
        title.setStyleSheet('font-size:22px; font-weight:bold')
        layout.addWidget(title)
        instructions = QtWidgets.QLabel(
            '이 창을 클릭한 뒤 W를 누르고 있으면 전진\n'
            'W를 떼거나 Space / Esc / 다른 창 클릭 → 정지\n'
            '방향은 실제 IMU를 돌려서 변경합니다.\n'
            'GPS 없음 · Local/Global은 상대 이동 추정입니다.')
        layout.addWidget(instructions)
        self.motion_label = QtWidgets.QLabel()
        self.motion_label.setStyleSheet('font-size:21px; font-weight:bold; padding:8px')
        layout.addWidget(self.motion_label)
        self.readout = QtWidgets.QLabel()
        self.readout.setStyleSheet('font-size:15px')
        layout.addWidget(self.readout)
        forward = QtWidgets.QPushButton('누르는 동안 전진 (W)')
        forward.setFocusPolicy(QtCore.Qt.NoFocus)
        forward.setMinimumHeight(45)
        forward.pressed.connect(lambda: self.set_held(True))
        forward.released.connect(self.stop)
        layout.addWidget(forward)
        stop = QtWidgets.QPushButton('정지 (Space)')
        stop.setFocusPolicy(QtCore.Qt.NoFocus)
        stop.clicked.connect(self.stop)
        layout.addWidget(stop)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        QtWidgets.QApplication.instance().applicationStateChanged.connect(
            lambda state: self.stop() if state != QtCore.Qt.ApplicationActive else None)

    def store(self, name, value):
        with self.lock:
            self.values[name] = value

    def imu(self, message):
        # Do not treat delayed/stale samples as permission to move.
        age = (rospy.Time.now() - message.header.stamp).to_sec()
        q = message.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        with self.lock:
            if -.05 <= age <= .25 and math.isfinite(norm) and abs(norm - 1.) <= .001:
                self.imu_receipt = time.monotonic()
                self.imu_stamp = message.header.stamp
            self.values['raw_yaw'] = yaw(message.orientation)

    def set_held(self, held):
        self.held = held
        self.tick()

    def stop(self):
        self.set_held(False)

    def tick(self):
        if rospy.is_shutdown():
            self.close()
            return
        now = time.monotonic()
        self.distance += self.previous_speed * max(0., now - self.previous_time)
        self.previous_time = now
        with self.lock:
            values = dict(self.values)
            fresh = self.imu_receipt is not None and now - self.imu_receipt <= .3
            fresh = fresh and -.05 <= (rospy.Time.now() - self.imu_stamp).to_sec() <= .25
        if not fresh:
            self.held = False
        speed = self.speed if self.held and self.isActiveWindow() and fresh else 0.
        self.previous_speed = speed
        self.alive = (self.alive + 1) % 256
        message = SerialFeedBack()
        message.speed = speed
        message.encoder = self.delta if speed else 0
        message.alive = self.alive
        self.publisher.publish(message)
        motion = '전진' if speed else '정지'
        self.motion_label.setText('{}  {:.1f} m/s · 입력 거리 {:.2f} m'.format(motion, speed, self.distance))
        self.motion_label.setStyleSheet('font-size:21px;font-weight:bold;padding:8px;color:'
                                       + ('#15803d' if speed else '#475569'))
        lines = ['IMU: ' + ('실시간 수신' if fresh else '미수신 / 오래된 입력 → 정지')]
        for key, title in [('raw_yaw', 'Raw yaw'), ('calibrated_yaw', 'Calibrated yaw')]:
            if key in values:
                lines.append('{}: {:.2f}°'.format(title, values[key]))
        for key in ('local', 'global'):
            if key in values:
                x, y, angle = values[key]
                lines.append('{}: ({:.2f}, {:.2f}) m / {:.2f}°'.format(key.title(), x, y, angle))
        self.readout.setText('\n'.join(lines))
        self.status.publish(String(json.dumps(dict(speed_mps=speed, encoder_delta=message.encoder,
            alive=self.alive, imu_fresh=fresh, held=self.held, input_distance_m=self.distance,
            values=values))))

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_W:
            if not event.isAutoRepeat():
                self.set_held(True)
            event.accept()
        elif event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
            self.stop()
            event.accept()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() == QtCore.Qt.Key_W:
            if not event.isAutoRepeat():
                self.stop()
            event.accept()
        else:
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        self.stop()
        super().focusOutEvent(event)

    def closeEvent(self, event):
        self.timer.stop()
        if not rospy.is_shutdown():
            self.stop()
            rospy.signal_shutdown('keyboard test closed')
        event.accept()


if __name__ == '__main__':
    rospy.init_node('keyboard_encoder_test', disable_signals=True)
    app = QtWidgets.QApplication(sys.argv)
    window = KeyboardEncoder()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    window.activateWindow()
    sys.exit(app.exec_())
