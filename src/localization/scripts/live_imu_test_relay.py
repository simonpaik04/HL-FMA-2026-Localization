#!/usr/bin/env python3
"""Copy an existing live IMU stream between ROS masters without sensor writes."""
import io
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
from urllib.parse import urlparse

import rospy
from genpy import DeserializationError
from sensor_msgs.msg import Imu


def source_reader(master, topic, fd):
    os.environ['ROS_MASTER_URI'] = master
    rospy.init_node('keyboard_test_imu_source', anonymous=True)
    pipe = os.fdopen(int(fd), 'wb', buffering=0)

    def send(message):
        data = io.BytesIO()
        message.serialize(data)
        payload = data.getvalue()
        try:
            pipe.write(struct.pack('<I', len(payload)) + payload)
        except (BrokenPipeError, OSError):
            rospy.signal_shutdown('test relay closed')

    rospy.Subscriber(topic, Imu, send, queue_size=10, tcp_nodelay=True)
    rospy.spin()


def exact_read(pipe, size):
    chunks = bytearray()
    while len(chunks) < size:
        chunk = pipe.read(size - len(chunks))
        if not chunk:
            raise EOFError('live IMU reader exited')
        chunks.extend(chunk)
    return bytes(chunks)


def main():
    rospy.init_node('live_imu_test_relay')
    source = rospy.get_param('~source_master_uri')
    target = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    # This bench tool deliberately requires separate local master ports.
    if urlparse(source).port == urlparse(target).port:
        raise ValueError('Use a separate ROS master port for the keyboard test')
    topic = rospy.get_param('~topic', '/mando_localization/internal/driver/imu')
    publisher = rospy.Publisher(topic, Imu, queue_size=10)
    read_fd, write_fd = os.pipe()
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), '--source-reader', source, topic, str(write_fd)],
        pass_fds=(write_fd,))
    os.close(write_fd)

    def forward():
        try:
            with os.fdopen(read_fd, 'rb') as pipe:
                while not rospy.is_shutdown():
                    size = struct.unpack('<I', exact_read(pipe, 4))[0]
                    if not 0 < size < 65536:
                        raise ValueError('invalid IMU pipe frame')
                    message = Imu().deserialize(exact_read(pipe, size))
                    # Preserve stamp, frame, orientation, gyro, acceleration and
                    # covariance. rospy assigns only a new publisher sequence.
                    publisher.publish(message)
        except (EOFError, OSError, ValueError, DeserializationError) as error:
            if not rospy.is_shutdown():
                rospy.logerr('Live IMU relay stopped: %s', error)
                rospy.signal_shutdown(str(error))

    def cleanup():
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3)

    rospy.on_shutdown(cleanup)
    threading.Thread(target=forward, daemon=True).start()
    rospy.loginfo('Read-only live IMU relay: %s -> %s (%s)', source, target, topic)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--source-reader':
        source_reader(*sys.argv[2:])
    else:
        main()
