#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_base / nav2 失败时的兜底节点（ROS2/rclpy 版）。

触发条件（同时满足才接管）：
  - nav2 的 /cmd_vel 已经停发超过 nav_stale_s 秒（说明规划器卡住或被 abort）；
  - 最近收到过 /goal_pose（RViz "2D Goal Pose" 发的话题）；
  - 当前不在目标点附近。
接管期间：
  - 用 tf2 查目标点相对 base_link 的方位角，朝目标修角；
  - 前向 120 度扇区 1 m 内有障碍就停车，往空间大的一侧转；
  - 输出发到 /simple_nav/cmd_vel，不直接发 /uav/cmd_vel。
nav2 恢复发 /cmd_vel 后，本节点自动闭嘴，由 nav_relay 仲裁。
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from . import nav_logic


class SimpleNav(Node):
    def __init__(self):
        super().__init__("simple_nav")
        self.declare_parameter("nav_stale_s", 1.5)
        self.declare_parameter("scan_timeout_s", 0.5)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("half_sector", np.deg2rad(60.0))

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, "/simple_nav/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.nav_vel_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        self._last_scan = None
        self._last_scan_t = None
        self._last_nav_t = None
        self._goal = None
        self._last_takeover_t = None

        self.timer = self.create_timer(0.1, self.tick)

    def scan_cb(self, msg: LaserScan):
        self._last_scan = msg
        self._last_scan_t = self.get_clock().now()

    def nav_vel_cb(self, _msg: Twist):
        self._last_nav_t = self.get_clock().now()

    def goal_cb(self, msg: PoseStamped):
        self._goal = msg

    def nav2_alive(self) -> bool:
        if self._last_nav_t is None:
            return False
        return (self.get_clock().now() - self._last_nav_t).nanoseconds * 1e-9 \
            < float(self.get_parameter("nav_stale_s").value)

    def goal_bearing(self):
        if self._goal is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("base_frame").value,
                self._goal.header.frame_id or self.get_parameter("goal_frame").value,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        if np.hypot(dx, dy) < 0.3:
            return None  # 到了
        return float(np.arctan2(dy, dx))

    def tick(self):
        if self.nav2_alive():
            self._last_takeover_t = None
            return
        if self._last_scan is None:
            return
        if self._last_scan_t is None or \
                (self.get_clock().now() - self._last_scan_t).nanoseconds * 1e-9 > \
                float(self.get_parameter("scan_timeout_s").value):
            return

        scan = self._last_scan
        half = float(self.get_parameter("half_sector").value)
        rmax = float(scan.range_max)
        front = nav_logic.front_min_range(scan.ranges, scan.angle_min,
                                          scan.angle_increment, half,
                                          range_max=rmax)
        left, right = nav_logic.side_min_range(scan.ranges, scan.angle_min,
                                                scan.angle_increment, half,
                                                range_max=rmax)
        vx, wz = nav_logic.decide_command(front, left, right, self.goal_bearing())
        if vx == 0.0 and wz == 0.0:
            return

        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)
        self._last_takeover_t = self.get_clock().now()


def main():
    rclpy.init()
    node = SimpleNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
