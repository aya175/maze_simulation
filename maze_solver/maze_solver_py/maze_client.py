#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from maze_solver.action import MoveRobotX, MoveRobotYaw

HALF_PI = math.pi / 2.0

# Official maze service uses up=2.0 / down=0.0. Command 2.0 so the slab
# goes to the joint maximum and stays there (do not mix with 1.0).
WALL_UP = 2.0
WALL_DOWN = 0.0
WALL_HOLD_PERIOD = 0.05
WALL_OPEN_SEC = 6.0
WALL_RISE_SEC = 5.0
WALL_FALL_SEC = 5.0


class MazeClient(Node):

    def __init__(self):
        super().__init__('maze_client')

        self.declare_parameter('wall_service', '/toggle_walls_1_2')
        self.declare_parameter('wall_open_sec', WALL_OPEN_SEC)

        wall_service = self.get_parameter('wall_service').value
        self.wall_open_sec = float(self.get_parameter('wall_open_sec').value)

        self._move_x_client = ActionClient(self, MoveRobotX, 'move_robot_x')
        self._move_yaw_client = ActionClient(self, MoveRobotYaw, 'move_robot_yaw')
        self._wall_client = self.create_client(SetBool, wall_service)

        self._pub_wall_1 = self.create_publisher(Float64, '/wall_1/cmd_pos', 10)
        self._pub_wall_2 = self.create_publisher(Float64, '/wall_2/cmd_pos', 10)
        self._hold_wall1 = WALL_DOWN
        self._hold_wall2 = WALL_DOWN
        self._hold_enabled = True
        self._sim_nsec = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._hold_timer = self.create_timer(WALL_HOLD_PERIOD, self._publish_wall_hold)

        self.get_logger().info(f"maze_client ready (wall service: {wall_service}).")

    def _on_clock(self, msg: Clock):
        self._sim_nsec = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)

    def _publish_wall_hold(self):
        if not self._hold_enabled:
            return
        self._pub_wall_1.publish(Float64(data=float(self._hold_wall1)))
        self._pub_wall_2.publish(Float64(data=float(self._hold_wall2)))

    def hold_targets(self, wall1: float, wall2: float) -> None:
        self._hold_wall1 = float(wall1)
        self._hold_wall2 = float(wall2)
        self._hold_enabled = True
        self._publish_wall_hold()

    def wait_sim(self, seconds: float, label: str = '') -> None:
        """Wait `seconds` of Gazebo time (falls back to a long wall-clock wait)."""
        seconds = float(seconds)
        if label:
            self.get_logger().info(label)
        wall_cap = time.monotonic() + max(seconds * 12.0, 25.0)

        while rclpy.ok() and self._sim_nsec is None:
            if time.monotonic() > wall_cap:
                self.get_logger().warn("No /clock; waiting in real time.")
                end = time.monotonic() + seconds
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                return
            rclpy.spin_once(self, timeout_sec=0.05)

        start = self._sim_nsec
        target = int(seconds * 1_000_000_000)
        while rclpy.ok() and (self._sim_nsec - start) < target:
            if time.monotonic() > wall_cap:
                self.get_logger().warn("Sim-time wait hit real-time cap.")
                break
            rclpy.spin_once(self, timeout_sec=0.05)

    def raise_only(self, wall: int) -> None:
        """Send ONE wall to full up (2.0). The other is pinned down. No XOR after this."""
        if wall == 1:
            self.hold_targets(WALL_UP, WALL_DOWN)
            self.call_toggle_walls(True)
            # Pin immediately so the XOR burst cannot drop wall 1 or raise wall 2 later.
            self.hold_targets(WALL_UP, WALL_DOWN)
        else:
            self.hold_targets(WALL_DOWN, WALL_UP)
            self.call_toggle_walls(False)
            self.hold_targets(WALL_DOWN, WALL_UP)

    def lower_both(self) -> None:
        self.hold_targets(WALL_DOWN, WALL_DOWN)

    def wall_loop_once(self, wall: int, drive_distance: float) -> bool:
        """Open this wall to the top, stay fully up 6 s, drive through, then close.

        The other wall stays down the whole time (no open/close of both together).
        """
        name = 'first' if wall == 1 else 'second'
        self.get_logger().info(f"{name} red wall: going FULLY UP.")
        self.raise_only(wall)
        self.wait_sim(WALL_RISE_SEC, f"{name} red wall: rising to the top...")

        self.get_logger().info(
            f"{name} red wall: at the top, holding FULLY UP for {self.wall_open_sec:.0f} s (not lowering)."
        )
        self.wait_sim(self.wall_open_sec)

        if not self.move_x(drive_distance):
            return False

        self.get_logger().info(f"{name} red wall: time over, now closing.")
        self.lower_both()
        self.wait_sim(WALL_FALL_SEC, f"{name} red wall: going fully down...")
        return True

    def move_x(self, distance: float, server_wait_timeout: float = 30.0) -> bool:
        self.get_logger().info(f"move_x: {distance:.3f} m")

        if not self._move_x_client.wait_for_server(timeout_sec=server_wait_timeout):
            self.get_logger().error("move_robot_x action server not available.")
            return False

        goal_msg = MoveRobotX.Goal()
        goal_msg.distance = float(distance)

        send_future = self._move_x_client.send_goal_async(
            goal_msg, feedback_callback=self._move_x_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("move_robot_x goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("move_robot_x returned no result.")
            return False
        result = wrapped.result
        self.get_logger().info(f"move_x result: {result.message}")
        return bool(result.success)

    def move_yaw(self, target_yaw: float, server_wait_timeout: float = 30.0) -> bool:
        self.get_logger().info(f"move_yaw: {target_yaw:.3f} rad")

        if not self._move_yaw_client.wait_for_server(timeout_sec=server_wait_timeout):
            self.get_logger().error("move_robot_yaw action server not available.")
            return False

        goal_msg = MoveRobotYaw.Goal()
        goal_msg.target_yaw = float(target_yaw)

        send_future = self._move_yaw_client.send_goal_async(
            goal_msg, feedback_callback=self._move_yaw_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("move_robot_yaw goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("move_robot_yaw returned no result.")
            return False
        result = wrapped.result
        self.get_logger().info(f"move_yaw result: {result.message}")
        return bool(result.success)

    def call_toggle_walls(self, data: bool, server_wait_timeout: float = 8.0) -> bool:
        if not self._wall_client.wait_for_service(timeout_sec=server_wait_timeout):
            self.get_logger().error("/toggle_walls_1_2 service not available.")
            return False

        req = SetBool.Request()
        req.data = bool(data)
        future = self._wall_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("toggle_walls: no response.")
            return False
        self.get_logger().info(f"toggle_walls({data}): {response.message}")
        return bool(response.success)

    def _move_x_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f"move_x feedback: travelled={fb.distance_travelled:.3f}, "
            f"remaining={fb.distance_remaining:.3f}"
        )

    def _move_yaw_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f"move_yaw feedback: turned={fb.yaw_turned:.3f}, "
            f"remaining={fb.yaw_remaining:.3f}"
        )

    def solve_maze(self) -> bool:
        self.get_logger().info("solve_maze: waiting for action servers and wall service...")

        if not self._move_x_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("move_robot_x server not available.")
            return False
        if not self._move_yaw_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("move_robot_yaw server not available.")
            return False
        if not self._wall_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/toggle_walls_1_2 not available. Is the maze sim running?")
            return False

        if not self.move_yaw(HALF_PI):
            return False

        # Loop: wall 1 fully up → hold 6 s → close, then wall 2 fully up → hold 6 s → close.
        for wall, distance in ((1, 1.05), (2, 1.10)):
            if not self.wall_loop_once(wall, distance):
                return False

        if not self.move_yaw(-HALF_PI):
            return False

        if not self.move_x(4.55):
            return False

        self.get_logger().info("solve_maze: sequence complete.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MazeClient()
    try:
        success = node.solve_maze()
        if success:
            node.get_logger().info("Maze solved successfully!")
        else:
            node.get_logger().error("Maze run did not complete successfully.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
