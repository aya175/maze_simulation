#!/usr/bin/env python3
import math
import threading
import time

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quaternion(q) -> float:
    """Yaw (radians) from geometry_msgs/Quaternion, Z-up."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomWatcher:
    def __init__(self, node, topic: str = '/odom', callback_group=None):
        self._node = node
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_msg_time = None
        self._received_at_least_once = False

        kwargs = {}
        if callback_group is not None:
            kwargs['callback_group'] = callback_group

        # Sensor-data QoS (BEST_EFFORT) matches typical Gazebo/ros_gz publishers.
        self._sub = node.create_subscription(
            Odometry, topic, self._callback, qos_profile_sensor_data, **kwargs)

    def _callback(self, msg: Odometry):
        with self._lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            self._last_msg_time = time.monotonic()
            self._received_at_least_once = True

    @property
    def has_data(self) -> bool:
        with self._lock:
            return self._received_at_least_once

    def pose(self):
        """Return (x, y, yaw) as a consistent snapshot."""
        with self._lock:
            return self._x, self._y, self._yaw

    @property
    def x(self) -> float:
        with self._lock:
            return self._x

    @property
    def y(self) -> float:
        with self._lock:
            return self._y

    @property
    def yaw(self) -> float:
        with self._lock:
            return self._yaw

    def seconds_since_last_message(self) -> float:
        with self._lock:
            if self._last_msg_time is None:
                return float('inf')
            return time.monotonic() - self._last_msg_time
