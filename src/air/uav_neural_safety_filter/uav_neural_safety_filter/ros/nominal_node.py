#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""演示用名义控制器：朝固定目标点飞，输出机头系速度到 /uav/cmd_vel_nominal。

安全层只关心"标称指令是什么"，真实部署时这个节点可以换成键盘、规划器或
其他策略；只要把指令发到同一个话题即可。
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class NominalNode(Node):
    def __init__(self):
        super().__init__("uav_nominal_controller")
        p = self.declare_parameter
        p("goal_x", 8.0)
        p("goal_y", 8.0)
        p("goal_z", 1.6)
        p("speed", 2.5)
        p("rate_hz", 20.0)
        p("topic.odom", "/uav/odom")
        p("topic.cmd", "/uav/cmd_vel_nominal")
        g = lambda k: self.get_parameter(k).value            # noqa: E731
        self.goal = np.array([float(g("goal_x")), float(g("goal_y")),
                              float(g("goal_z"))])
        self.speed = float(g("speed"))
        self.pos = np.zeros(3)
        self.yaw = 0.0
        self.pub = self.create_publisher(Twist, str(g("topic.cmd")), 10)
        self.create_subscription(Odometry, str(g("topic.odom")), self.on_odom, 10)
        self.create_timer(1.0 / max(float(g("rate_hz")), 1e-3), self.tick)

    def on_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def tick(self) -> None:
        dp = self.goal - self.pos
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx = c * dp[0] + s * dp[1]
        dy = -s * dp[0] + c * dp[1]
        n = math.hypot(dx, dy)
        u = np.zeros(3)
        if n > 0.5:
            u[0] = self.speed * dx / n
            u[1] = self.speed * dy / n
        u[2] = float(np.clip(0.8 * dp[2], -0.8, 0.8))
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = (float(v) for v in u)
        self.pub.publish(t)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = NominalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
