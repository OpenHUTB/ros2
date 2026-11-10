#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""演示用标称控制器：只朝目标飞，不做任何避障。

它存在的意义是给安全层一个“会撞”的上游：把指令发到
``/uav/cmd_vel_nominal``，由安全层修正后转发到 ``/uav/cmd_vel``。
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

__all__ = ["NominalNode", "main"]


class NominalNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_nominal_controller")
        p = self.declare_parameter
        self.topic_odom = str(p("topic.odom", "/uav/odom").value)
        self.topic_cmd = str(p("topic.cmd_out", "/uav/cmd_vel_nominal").value)
        self.topic_goal = str(p("topic.goal", "/uav/goal").value)
        self.rate_hz = float(p("rate_hz", 20.0).value)
        self.speed = float(p("speed", 1.8).value)
        self.pos = None
        self.yaw = 0.0
        self.goal = np.array([6.0, 0.0, 1.6])
        self.create_subscription(Odometry, self.topic_odom, self.on_odom, 10)
        self.create_subscription(Point, self.topic_goal, self.on_goal, 10)
        self.pub = self.create_publisher(Twist, self.topic_cmd, 10)
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-3), self.tick)
        self.get_logger().info("标称控制器发布到 %s" % self.topic_cmd)

    def on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z], float)
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_goal(self, m: Point) -> None:
        self.goal = np.array([m.x, m.y, m.z], float)

    def tick(self) -> None:
        if self.pos is None:
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        dp = rot.T @ (self.goal - self.pos)
        d = max(float(np.hypot(dp[0], dp[1])), 1e-6)
        msg = Twist()
        msg.linear.x = self.speed * dp[0] / d
        msg.linear.y = self.speed * dp[1] / d
        msg.linear.z = float(np.clip(0.8 * dp[2], -0.8, 0.8))
        self.pub.publish(msg)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = NominalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:                            # pragma: no cover
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                                # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":                               # pragma: no cover
    raise SystemExit(main())
