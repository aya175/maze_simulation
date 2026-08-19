#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist

from maze_solver.action import MoveRobotX, MoveRobotYaw
from maze_solver_py.odom_watcher import OdomWatcher


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def scaled_speed(remaining: float, max_speed: float, min_speed: float, slow_zone: float) -> float:
    """Full speed until near the target, then taper so we do not overshoot."""
    remaining = max(remaining, 0.0)
    if remaining >= slow_zone or slow_zone <= 0.0:
        return max_speed
    return max(min_speed, max_speed * (remaining / slow_zone))


class MazeActionServers(Node):
    def __init__(self):
        super().__init__('maze_action_servers')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('linear_min_speed', 0.05)
        self.declare_parameter('linear_slow_zone', 0.12)
        self.declare_parameter('distance_tolerance', 0.03)
        self.declare_parameter('angular_speed', 0.7)
        self.declare_parameter('angular_min_speed', 0.15)
        self.declare_parameter('angular_slow_zone', 0.35)
        self.declare_parameter('yaw_tolerance', 0.04)
        self.declare_parameter('odom_wait_timeout', 8.0)
        self.declare_parameter('odom_stale_timeout', 5.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('timeout_multiplier', 20.0)
        self.declare_parameter('timeout_buffer_sec', 40.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.linear_min_speed = float(self.get_parameter('linear_min_speed').value)
        self.linear_slow_zone = float(self.get_parameter('linear_slow_zone').value)
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.angular_min_speed = float(self.get_parameter('angular_min_speed').value)
        self.angular_slow_zone = float(self.get_parameter('angular_slow_zone').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.odom_wait_timeout = float(self.get_parameter('odom_wait_timeout').value)
        self.odom_stale_timeout = float(self.get_parameter('odom_stale_timeout').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timeout_multiplier = float(self.get_parameter('timeout_multiplier').value)
        self.timeout_buffer_sec = float(self.get_parameter('timeout_buffer_sec').value)

        self._odom_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom = OdomWatcher(self, self.odom_topic, callback_group=self._odom_group)

        self._move_x_server = ActionServer(
            self, MoveRobotX, 'move_robot_x',
            execute_callback=self.execute_x,
            goal_callback=self.goal_x,
            cancel_callback=self.cancel_cb,
            callback_group=self._action_group,
        )
        self._move_yaw_server = ActionServer(
            self, MoveRobotYaw, 'move_robot_yaw',
            execute_callback=self.execute_yaw,
            goal_callback=self.goal_yaw,
            cancel_callback=self.cancel_cb,
            callback_group=self._action_group,
        )

        self.get_logger().info(
            f"Action servers ready: move_robot_x, move_robot_yaw "
            f"(cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic})"
        )

    def goal_x(self, goal_request):
        self.get_logger().info(f"move_robot_x goal: distance={goal_request.distance:.3f} m")
        return GoalResponse.ACCEPT

    def goal_yaw(self, goal_request):
        self.get_logger().info(f"move_robot_yaw goal: target_yaw={goal_request.target_yaw:.3f} rad")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    def stop_robot(self):
        msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.02)

    def wait_for_odom(self):
        wait_start = time.monotonic()
        while not self.odom.has_data:
            if time.monotonic() - wait_start > self.odom_wait_timeout:
                return False
            time.sleep(0.05)
        return True

    def motion_timeout(self, magnitude: float, speed: float) -> float:
        expected = magnitude / max(speed, 1e-6)
        return expected * self.timeout_multiplier + self.timeout_buffer_sec

    def abort_if_stale_or_timeout(self, goal_handle, action_start, max_time, label: str):
        if self.odom.seconds_since_last_message() > self.odom_stale_timeout:
            self.get_logger().error(
                f"Aborting {label}: /odom silent for >{self.odom_stale_timeout}s."
            )
            self.stop_robot()
            goal_handle.abort()
            return f"Aborted: /odom went silent mid-action ({label})."
        if time.monotonic() - action_start > max_time:
            self.get_logger().error(
                f"Aborting {label}: timeout after {max_time:.1f}s."
            )
            self.stop_robot()
            goal_handle.abort()
            return f"Aborted: timed out ({label})."
        return None

    def execute_x(self, goal_handle):
        target = goal_handle.request.distance
        direction = 1.0 if target >= 0.0 else -1.0
        target_abs = abs(target)
        result = MoveRobotX.Result()
        period = 1.0 / max(self.control_rate_hz, 1.0)

        if not self.wait_for_odom():
            self.get_logger().error("Aborting move_robot_x: no /odom (proprioceptive sensor missing).")
            self.stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = "Aborted: /odom sensor not publishing."
            result.final_distance_travelled = 0.0
            return result

        start_x, start_y, _ = self.odom.pose()
        max_time = self.motion_timeout(target_abs, self.linear_speed)
        action_start = time.monotonic()
        distance_travelled = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled by client."
                result.final_distance_travelled = distance_travelled
                return result

            fail = self.abort_if_stale_or_timeout(goal_handle, action_start, max_time, 'move_robot_x')
            if fail:
                result.success = False
                result.message = fail
                result.final_distance_travelled = distance_travelled
                return result

            x, y, _ = self.odom.pose()
            distance_travelled = math.hypot(x - start_x, y - start_y)
            remaining = target_abs - distance_travelled

            feedback = MoveRobotX.Feedback()
            feedback.distance_travelled = distance_travelled
            feedback.distance_remaining = max(remaining, 0.0)
            goal_handle.publish_feedback(feedback)

            if remaining <= self.distance_tolerance:
                break

            cmd = Twist()
            cmd.linear.x = direction * scaled_speed(
                remaining, self.linear_speed, self.linear_min_speed, self.linear_slow_zone)
            self.cmd_vel_pub.publish(cmd)
            time.sleep(period)

        self.stop_robot()
        goal_handle.succeed()
        result.success = True
        result.message = f"Reached target distance ({distance_travelled:.3f} m)."
        result.final_distance_travelled = distance_travelled
        self.get_logger().info(result.message)
        return result

    def execute_yaw(self, goal_handle):
        target_yaw = goal_handle.request.target_yaw
        direction = 1.0 if target_yaw >= 0.0 else -1.0
        target_abs = abs(target_yaw)
        result = MoveRobotYaw.Result()
        period = 1.0 / max(self.control_rate_hz, 1.0)

        if not self.wait_for_odom():
            self.get_logger().error("Aborting move_robot_yaw: no /odom (proprioceptive sensor missing).")
            self.stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = "Aborted: /odom sensor not publishing."
            result.final_yaw_turned = 0.0
            return result

        _, _, last_yaw = self.odom.pose()
        yaw_turned = 0.0
        max_time = self.motion_timeout(target_abs, self.angular_speed)
        action_start = time.monotonic()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled by client."
                result.final_yaw_turned = yaw_turned
                return result

            fail = self.abort_if_stale_or_timeout(goal_handle, action_start, max_time, 'move_robot_yaw')
            if fail:
                result.success = False
                result.message = fail
                result.final_yaw_turned = yaw_turned
                return result

            _, _, current_yaw = self.odom.pose()
            yaw_turned += normalize_angle(current_yaw - last_yaw)
            last_yaw = current_yaw
            remaining = target_abs - abs(yaw_turned)

            feedback = MoveRobotYaw.Feedback()
            feedback.yaw_turned = yaw_turned
            feedback.yaw_remaining = max(remaining, 0.0)
            goal_handle.publish_feedback(feedback)

            if remaining <= self.yaw_tolerance:
                break

            cmd = Twist()
            cmd.angular.z = direction * scaled_speed(
                remaining, self.angular_speed, self.angular_min_speed, self.angular_slow_zone)
            self.cmd_vel_pub.publish(cmd)
            time.sleep(period)

        self.stop_robot()
        goal_handle.succeed()
        result.success = True
        result.message = f"Reached target yaw ({yaw_turned:.3f} rad)."
        result.final_yaw_turned = yaw_turned
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MazeActionServers()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
