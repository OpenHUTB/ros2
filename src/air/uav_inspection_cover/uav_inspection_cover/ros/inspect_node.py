#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Inspection progress node: odom in, target viewpoint + progress JSON out.

节点是薄壳：启动时用 core 把指定 case 的航线规划好，之后每个 odom
回调只调 core 的纯函数 ``progress_snapshot`` 并把结果以 JSON 发布。
"""
from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_inspection_cover.core.geometry import Params
from uav_inspection_cover.core.planner import build_cases, make_route, progress_snapshot


class InspectNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_inspection_cover_node")
        self.declare_parameter("case", "case_grid_short")
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("arrive_radius_m", 12.0)
        self.declare_parameter("publish_period_s", 0.5)

        case = str(self.get_parameter("case").value)
        self.radius = float(self.get_parameter("arrive_radius_m").value)
        period = float(self.get_parameter("publish_period_s").value)

        self.params = Params()
        corridor = build_cases()[case]
        self.route = make_route(corridor, self.params)
        self.hint = 0
        self.last: dict = {}

        self.create_subscription(Odometry, str(self.get_parameter(
            "odom_topic").value), self._on_odom, 10)
        self.pub = self.create_publisher(String, "/uav_inspection_cover/target", 10)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "route ready: %s, %d waypoints, %d viewpoints"
            % (case, len(self.route.waypoints), len(self.route.viewpoints)))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        # 判定逻辑在 core.planner（纯函数，有单元测试覆盖）
        self.last.update(progress_snapshot(
            self.route.waypoints, (p.x, p.y, p.z), self.radius, self.hint))
        self.hint = self.last.get("target_index", 0)

    def _tick(self) -> None:
        state = self.last or dict(target_index=0,
                                  target=[round(float(v), 2)
                                          for v in self.route.waypoints[0]],
                                  progress=0.0, done=False)
        self.pub.publish(String(data=json.dumps(state, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
