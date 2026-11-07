#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线调度节点：订阅各机里程计，周期发布任务分配表与预计到达时刻。

参数（``config/uav_time_window.yaml``）::

    uav_ids: ["UAV-1", "UAV-2", "UAV-3"]
    odom_prefix: "/uav"          # 话题名为 <prefix><n>/odom（单机为 <prefix>/odom）
    lambda_balance: 0.5
    default_speed: 8.0
    default_endurance: 420.0
    service_time: 15.0
    hover_weight: 0.2
    period: 1.0                  # s，重排周期

发布::

    /uav_time_window/schedule   std_msgs/String   JSON 分配表（assign + eta + 延误）

本节点是**薄壳**：任务表来自内置场景，判定逻辑全部在 core.assign
（``schedule_from_positions``，无 ROS 依赖、有单元测试覆盖）。收到全部
里程计前不发布，避免用缺省位置排出误导性分配。
"""
from __future__ import annotations

import json
from typing import Dict, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_time_window.core.assign import schedule_from_positions
from uav_time_window.core.mission import Config
from uav_time_window.core.scenarios import SCENARIOS, get_scenario


class ScheduleNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_time_window_schedule")
        self.declare_parameter("uav_ids", ["UAV-1", "UAV-2", "UAV-3"])
        self.declare_parameter("odom_prefix", "/uav")
        self.declare_parameter("scenario", "tight")
        self.declare_parameter("lambda_balance", 0.5)
        self.declare_parameter("default_speed", 8.0)
        self.declare_parameter("default_endurance", 420.0)
        self.declare_parameter("service_time", 15.0)
        self.declare_parameter("hover_weight", 0.2)
        self.declare_parameter("period", 1.0)

        self.ids = list(self.get_parameter("uav_ids").value)
        prefix = str(self.get_parameter("odom_prefix").value)
        scenario = str(self.get_parameter("scenario").value)
        self.cfg = Config(
            lambda_balance=float(self.get_parameter("lambda_balance").value),
            default_speed=float(self.get_parameter("default_speed").value),
            default_endurance=float(self.get_parameter("default_endurance").value),
            service_time=float(self.get_parameter("service_time").value),
            hover_weight=float(self.get_parameter("hover_weight").value),
        )
        self.tasks = get_scenario(scenario).tasks if scenario in SCENARIOS else []
        self.speeds = {u.uid: u.speed for u in
                       (get_scenario(scenario).uavs if self.tasks else [])}

        # 单机时订阅 <prefix>/odom；多机时按 <prefix>1/odom ... 逐机订阅
        if len(self.ids) == 1:
            topics = [("%s/odom" % prefix.rstrip("/"), self.ids[0])]
        else:
            topics = [("%s%d/odom" % (prefix.rstrip("/"), i + 1), uid)
                      for i, uid in enumerate(self.ids)]
        for topic, uid in topics:
            self.create_subscription(
                Odometry, topic,
                lambda msg, uid=uid: self._on_odom(uid, msg), 10)

        self.pos: Dict[str, Tuple[float, float]] = {}
        self.revision = 0
        self.pub = self.create_publisher(String, "/uav_time_window/schedule", 10)
        period = float(self.get_parameter("period").value)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "在线调度已启动：机队 %s，场景 %s（%d 个任务），周期 %.1f s"
            % (",".join(self.ids), scenario, len(self.tasks), period))

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, uid: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos[uid] = (p.x, p.y)

    def _tick(self) -> None:
        if set(self.ids) - set(self.pos):
            return                      # 还有飞机位置未知，不发布
        table = schedule_from_positions(self.pos, self.tasks, self.cfg,
                                        speeds=self.speeds)
        table["revision"] = self.revision
        self.revision += 1
        self.pub.publish(String(data=json.dumps(table, ensure_ascii=False)))
        self.get_logger().info("发布分配表 rev=%d：%s"
                               % (self.revision,
                                  {k: v for k, v in table["assign"].items()}))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScheduleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
