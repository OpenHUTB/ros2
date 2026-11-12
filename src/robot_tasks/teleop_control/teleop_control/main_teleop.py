#!/usr/bin/env python3
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from teleop_control.teleop_logic import TeleopCommandState, decode_key


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("poll_period_sec", 0.05)
        self.poll_period_sec = float(self.get_parameter("poll_period_sec").value)
        if not math.isfinite(self.poll_period_sec) or self.poll_period_sec <= 0.0:
            raise ValueError("poll_period_sec must be finite and greater than zero")

        self.command_state = TeleopCommandState(
            linear_speed=self.get_parameter("linear_speed").value,
            angular_speed=self.get_parameter("angular_speed").value,
            command_timeout_sec=self.get_parameter("command_timeout_sec").value,
        )
        if not sys.stdin.isatty():
            raise RuntimeError(
                "No interactive terminal detected. Run main_teleop in a terminal."
            )
        self.get_logger().info(
            "Keyboard control ready: W/S move, A/D turn, Space stops, Q exits."
        )

    def _publish(self, velocity):
        message = Twist()
        message.linear.x, message.angular.z = velocity
        self.publisher_.publish(message)
        self.get_logger().info(
            f"Published: linear={velocity[0]:.2f}, angular={velocity[1]:.2f}",
            throttle_duration_sec=1.0,
        )

    def run(self):
        fd = sys.stdin.fileno()
        previous_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                readable, _, _ = select.select([sys.stdin], [], [], self.poll_period_sec)
                now = time.monotonic()
                if readable:
                    key = decode_key(os.read(fd, 1))
                    if key is None:
                        self.get_logger().info("Keyboard input closed; publishing zero velocity.")
                        break
                    if key == "q":
                        self.get_logger().info("Stop requested; publishing zero velocity.")
                        break
                    self._publish(self.command_state.handle_key(key, now))
                else:
                    expired = self.command_state.expire(now)
                    if expired is not None:
                        self.get_logger().info("Keyboard command timed out; stopping robot.")
                        self._publish(expired)
        finally:
            try:
                self._publish(self.command_state.stop())
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, previous_settings)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
