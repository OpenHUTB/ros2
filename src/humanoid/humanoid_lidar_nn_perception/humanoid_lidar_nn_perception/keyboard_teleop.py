"""Keyboard teleop: WASD publishes user target velocity on /cmd_vel."""
from __future__ import annotations
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

VX_STEP = 0.1
WZ_STEP = 0.2


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vx = 0.0
        self.wz = 0.0
        self.get_logger().info(
            "Teleop ready. W/S: forward/back, A/D: turn left/right, "
            "space: stop, X: quit."
        )

    def publish(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.angular.z = self.wz
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            ch = sys.stdin.read(1)
            if ch == "w":
                node.vx = min(node.vx + VX_STEP, 0.4)
            elif ch == "s":
                node.vx = max(node.vx - VX_STEP, -0.1)
            elif ch == "a":
                node.wz = min(node.wz + WZ_STEP, 1.0)
            elif ch == "d":
                node.wz = max(node.wz - WZ_STEP, -1.0)
            elif ch == " ":
                node.vx = 0.0
                node.wz = 0.0
            elif ch == "x" or ch == "q":
                break
            node.publish()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
