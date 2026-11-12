#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/uav/cmd_vel 的唯一发布者（ROS2/rclpy 版）。

仲裁：
  - 订阅 nav2 输出 /cmd_vel；
  - 订阅 simple_nav 输出 /simple_nav/cmd_vel；
  - 兜底新鲜 > nav2 新鲜 > 都不发；
  - 统一限幅，z 恒为 0（定高交给 bridge）。
Noetic bridge 那边订阅的是 /uav/cmd_vel，经 ros1_bridge 跨域后话题名一致。
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from . import nav_logic


class NavRelay(Node):
    def __init__(self):
        super().__init__("nav_relay")
        self.declare_parameter("max_vx", 0.8)
        self.declare_parameter("max_vy", 0.5)
        self.declare_parameter("max_yaw", 0.6)
        self.declare_parameter("stale_s", 0.5)

        self.pub = self.create_publisher(Twist, "/uav/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel", self.nav_cb, 10)
        self.create_subscription(Twist, "/simple_nav/cmd_vel", self.fb_cb, 10)

        self._nav = None
        self._fb = None
        self._last_zero_t = None
        self.timer = self.create_timer(0.1, self.tick)

    def nav_cb(self, msg: Twist):
        self._nav = (msg, self.get_clock().now())

    def fb_cb(self, msg: Twist):
        self._fb = (msg, self.get_clock().now())

    def tick(self):
        now = self.get_clock().now()
        stale = float(self.get_parameter("stale_s").value)

        def age(pair):
            return (now - pair[1]).nanoseconds * 1e-9 if pair else None

        choice = nav_logic.select_source(
            now.nanoseconds * 1e-9,
            self._nav[1].nanoseconds * 1e-9 if self._nav else None,
            self._fb[1].nanoseconds * 1e-9 if self._fb else None,
            stale,
        )

        if choice is None:
            return

        src = self._fb if choice == "fallback" else self._nav
        msg_in = src[0]
        vx, vy, _vz, wz = nav_logic.limit_twist(
            msg_in.linear.x, msg_in.linear.y, msg_in.angular.z,
            float(self.get_parameter("max_vx").value),
            float(self.get_parameter("max_vy").value),
            float(self.get_parameter("max_yaw").value),
        )
        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.linear.z = 0.0
        out.angular.z = wz
        self.pub.publish(out)


def main():
    rclpy.init()
    node = NavRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
