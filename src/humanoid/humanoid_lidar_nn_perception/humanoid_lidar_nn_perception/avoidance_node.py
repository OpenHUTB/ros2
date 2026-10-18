"""Avoidance node: fuses /obstacle_info with user /cmd_vel -> safe /cmd_vel_safe.

If no trained PPO weight is present, falls back to a simple reactive rule:
slow down and steer away from the closest side when front is blocked.
"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

MAX_RANGE = 8.0
FRONT_THRESHOLD = 1.5  # m


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__("avoidance_node")
        self.obstacle = np.zeros(4, dtype=np.float32)  # nearest, front, left, right
        self.user_cmd = np.zeros(2, dtype=np.float32)  # vx, wz

        self.create_subscription(Float32MultiArray, "/obstacle_info", self._obs_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.timer = self.create_timer(0.1, self._control)
        self.get_logger().info("AvoidanceNode ready (reactive fallback mode).")

    def _obs_cb(self, msg: Float32MultiArray):
        self.obstacle = np.array(msg.data, dtype=np.float32)

    def _cmd_cb(self, msg: Twist):
        self.user_cmd[0] = msg.linear.x
        self.user_cmd[1] = msg.angular.z

    def _control(self):
        front, left, right = self.obstacle[1], self.obstacle[2], self.obstacle[3]
        vx = self.user_cmd[0]
        wz = self.user_cmd[1]

        if front < FRONT_THRESHOLD:
            # slow down and steer toward the more open side
            vx = 0.0
            if left > right:
                wz = 0.8
            else:
                wz = -0.8

        out = Twist()
        out.linear.x = float(np.clip(vx, 0.0, 0.4))
        out.angular.z = float(np.clip(wz, -1.0, 1.0))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
