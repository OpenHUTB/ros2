#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线噪声告警节点：订阅多机里程计，实时估算敏感点 Lp 并发布超标告警。

参数（``config/params.yaml`` 的 node 段）::

    uav_ids: ["UAV-1", "UAV-2"]
    odom_prefix: "/uav"          # 话题名为 <prefix><n>/odom
    alert_level_lp: 65.0
    tick_period: 0.5
    stale_timeout: 1.0

发布::

    /uav_noise_footprint/alert   std_msgs/String   JSON 形式的敏感点声级与超标列表

本节点只做**监视与告警**，不修改任何航迹；航线降噪由离线/计划层完成，
这条边界与 uav_deconflict 的监视节点一致。判定逻辑全部在
``core.exposure.current_levels / exceedances``（无 ROS 依赖，有单元测试）。
"""
from __future__ import annotations

import json
from typing import Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_noise_footprint.core.exposure import (SensitivePoint, current_levels,
                                               exceedances)

#: 演示用敏感点配置（真实部署应从地图服务加载；与 config/params.yaml 对应）
DEFAULT_POINTS = [
    SensitivePoint("小区A", 800.0, 30.0, "residential", 1.2),
    SensitivePoint("小区B", 1200.0, -50.0, "residential", 1.0),
    SensitivePoint("小区C", 400.0, 70.0, "residential", 1.0),
    SensitivePoint("实验小学", 0.0, 150.0, "school", 2.5),
    SensitivePoint("中心医院", 0.0, 60.0, "hospital", 2.5),
]


class NoiseMonitor(Node):
    def __init__(self) -> None:
        super().__init__("uav_noise_monitor")
        self.declare_parameter("uav_ids", ["UAV-1", "UAV-2"])
        self.declare_parameter("odom_prefix", "/uav")
        self.declare_parameter("alert_level_lp", 65.0)
        self.declare_parameter("tick_period", 0.5)
        self.declare_parameter("stale_timeout", 1.0)

        ids = list(self.get_parameter("uav_ids").value)
        prefix = str(self.get_parameter("odom_prefix").value)
        self.alert_floor = float(self.get_parameter("alert_level_lp").value)
        self.stale = float(self.get_parameter("stale_timeout").value)
        self.points = DEFAULT_POINTS

        self.pos: Dict[str, tuple] = {}
        self.stamp: Dict[str, float] = {}

        if len(ids) == 1:
            topics = [("%s/odom" % prefix.rstrip("/"), ids[0])]
        else:
            topics = [("%s%d/odom" % (prefix.rstrip("/"), i + 1), uid)
                      for i, uid in enumerate(ids)]
        for topic, uid in topics:
            self.create_subscription(
                Odometry, topic,
                lambda msg, uid=uid: self._on_odom(uid, msg), 10)

        self.pub_alert = self.create_publisher(
            String, "/uav_noise_footprint/alert", 10)
        self.create_timer(float(self.get_parameter("tick_period").value),
                          self._tick)
        self.get_logger().info(
            "噪声告警已启动，机队 %s，告警下限 %.0f dB，敏感点 %d 个"
            % (",".join(ids), self.alert_floor, len(self.points)))

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, uid: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos[uid] = (p.x, p.y, p.z)
        self.stamp[uid] = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = {uid: p for uid, p in self.pos.items()
                 if now - self.stamp.get(uid, -1e9) <= self.stale}
        levels = current_levels(fresh, self.points)
        alerts = [a for a in exceedances(levels, self.points)
                  if a["lp"] > self.alert_floor]
        payload = dict(
            uav_count=len(fresh),
            levels={k: (round(v, 1) if v == v and abs(v) != float("inf")
                        else None) for k, v in levels.items()},
            alerts=alerts,
            alert_floor=self.alert_floor)
        self.pub_alert.publish(String(
            data=json.dumps(payload, ensure_ascii=False)))
        for a in alerts:
            self.get_logger().warn(
                "噪声超标：%s（%s）Lp %.1f dB > 限值 %.1f dB（超 %.1f dB）"
                % (a["point"], a["kind"], a["lp"], a["limit"], a["exceed"]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NoiseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
