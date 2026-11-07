#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Online dispatch node: odom + battery in, next target & swap advice out (JSON).

单行英文摘要 + 中文正文。
薄壳节点：订阅 ``<odom_topic>``（nav_msgs/Odometry）与 ``<battery_topic>``
（std_msgs/Float32，电量 0~1），启动时用 core 规划一次当前 case 的架次，
之后每个 tick 调 core.planner.decide_next 做判定，把下一目标站与建议
换电时机以 JSON 发布到 ``<dispatch_topic>``（std_msgs/String）。
判定逻辑全部在 core，节点只做话题搬运。
"""
from __future__ import annotations

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

from uav_delivery_route.core.cases import get_case
from uav_delivery_route.core.energy import EnergyModel
from uav_delivery_route.core.planner import RouteConfig, decide_next, plan_routes

#: 判定"已到站"的水平半径（m）
ARRIVE_RADIUS_M = 2.0


class DispatchNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_delivery_dispatch")
        self.declare_parameter("case", "case_farm_dense")
        self.declare_parameter("cruise_speed_ms", 9.0)
        self.declare_parameter("cruise_alt_m", 8.0)
        self.declare_parameter("hover_s", 20.0)
        self.declare_parameter("swap_s", 90.0)
        self.declare_parameter("payload_cap_kg", 2.5)
        self.declare_parameter("return_margin", 1.25)
        self.declare_parameter("n_sectors", 3)
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("battery_topic", "/uav/battery")
        self.declare_parameter("dispatch_topic", "/uav_delivery/dispatch")

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.cfg = RouteConfig(cruise_speed_ms=float(p("cruise_speed_ms")),
                               cruise_alt_m=float(p("cruise_alt_m")),
                               hover_s=float(p("hover_s")),
                               swap_s=float(p("swap_s")),
                               payload_cap_kg=float(p("payload_cap_kg")),
                               return_margin=float(p("return_margin")),
                               n_sectors=int(p("n_sectors")))
        self.model = EnergyModel()
        self.plan = plan_routes(get_case(str(p("case"))), self.cfg, self.model)

        self.pos = (0.0, 0.0)
        self.battery = 1.0
        self.visited: set = set()

        self.create_subscription(Odometry, str(p("odom_topic")), self._on_odom, 10)
        self.create_subscription(Float32, str(p("battery_topic")), self._on_battery, 10)
        self.pub = self.create_publisher(String, str(p("dispatch_topic")), 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            "dispatch ready: case=%s, %d sorties, topics odom=%s battery=%s out=%s"
            % (p("case"), len(self.plan.sorties), p("odom_topic"),
               p("battery_topic"), p("dispatch_topic")))

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.position
        self.pos = (q.x, q.y)

    def _on_battery(self, msg: Float32) -> None:
        self.battery = float(msg.data)

    def _tick(self) -> None:
        # 到站判定：水平距离进入半径即标记已服务
        d = decide_next(self.plan, self.visited, self.pos,
                        self.battery, self.model, self.cfg)
        tgt = d.get("target")
        if d["action"] == "deliver" and tgt is not None:
            if math.hypot(self.pos[0] - tgt["x"], self.pos[1] - tgt["y"]) < ARRIVE_RADIUS_M:
                self.visited.add(tgt["sid"])
                d = decide_next(self.plan, self.visited, self.pos, self.battery,
                                self.model, self.cfg)
                tgt = d.get("target")
        msg = String(data=json.dumps(dict(action=d["action"], sortie=d["sortie"],
                                          target=tgt, battery_frac=d["battery_frac"],
                                          pos=dict(x=self.pos[0], y=self.pos[1]),
                                          visited=sorted(self.visited)),
                                      ensure_ascii=False))
        self.pub.publish(msg)
        if tgt is not None:
            self.get_logger().info("%s -> %s (%.1f m), battery %.0f%%"
                                   % (d["action"], tgt["sid"],
                                      d["distance_to_target_m"], d["battery_frac"] * 100))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
