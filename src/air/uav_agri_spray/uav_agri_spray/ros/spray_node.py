#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Online spray progress node: odom in, line id + sprayed area % out (JSON).

在线作业进度节点：订阅里程计，实时发布当前扫描线号与已作业面积百分比
（JSON 字符串）。所有判定逻辑下沉 core.planner.progress_state（纯 numpy，
有单元测试覆盖），节点只做话题收发，是薄壳。
"""
from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_agri_spray.core.planner import SprayConfig, plan_field, progress_state
from uav_agri_spray.core.scenarios import case_outline, get_case


class SprayProgressNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_agri_spray_node")
        self.declare_parameter("case", "case_field_rect")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_topic", "/uav_agri_spray/status")
        self.declare_parameter("status_period_s", 0.5)
        self.declare_parameter("spray_width", 3.0)
        self.declare_parameter("overlap", 0.2)
        self.declare_parameter("speed", 4.0)
        self.declare_parameter("alt", 2.5)
        self.declare_parameter("obstacle_margin", 5.0)

        case_name = str(self.get_parameter("case").value)
        self.cfg = SprayConfig(
            spray_width=float(self.get_parameter("spray_width").value),
            overlap=float(self.get_parameter("overlap").value),
            speed=float(self.get_parameter("speed").value),
            alt=float(self.get_parameter("alt").value),
            obstacle_margin=float(self.get_parameter("obstacle_margin").value))
        case = get_case(case_name)
        self.path = plan_field(case.subregions, case.obstacles, self.cfg)
        self.outline = case_outline(case)
        self.latest = None

        odom = str(self.get_parameter("odom_topic").value)
        status = str(self.get_parameter("status_topic").value)
        period = float(self.get_parameter("status_period_s").value)
        self.create_subscription(Odometry, odom, self._on_odom, 10)
        self.pub = self.create_publisher(String, status, 10)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "spray progress on %s: %d lines, %.1f m plan; status -> %s"
            % (case_name, len(self.path.lines), self.path.total_length, status))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.latest = (p.x, p.y, p.z)

    def _tick(self) -> None:
        if self.latest is None:
            return
        state = progress_state(self.path, self.latest)
        self.pub.publish(String(data=json.dumps(state, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SprayProgressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
