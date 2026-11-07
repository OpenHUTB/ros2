#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线间隔监视节点：订阅多机里程计，实时发布间隔状态与最接近机对。

参数（``config/params.yaml``）::

    uav_ids: ["UAV-1", "UAV-2", ...]
    odom_prefix: "/uav"          # 话题名为 <prefix><n>/odom
    sep_horizontal: 50.0
    sep_vertical: 20.0
    stale_timeout: 1.0

发布::

    /uav_deconflict/status   std_msgs/String      JSON 形式的最小间隔与违规计数
    /uav_deconflict/markers  visualization_msgs/MarkerArray  最接近机对连线与文字

本节点只做**监视与告警**，不直接修改任何航迹；冲突解脱由离线/计划层完成，
线上改动航迹需要经过空域管理方的批准，这条边界是刻意保留的。
"""
from __future__ import annotations

import json
from typing import Dict

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from uav_deconflict.core.conflict import Separation
from uav_deconflict.core.monitor import Closest, closest, fresh_positions


class DeconflictMonitor(Node):
    def __init__(self) -> None:
        super().__init__("uav_deconflict_monitor")
        self.declare_parameter("uav_ids", ["UAV-1", "UAV-2"])
        self.declare_parameter("odom_prefix", "/uav")
        self.declare_parameter("sep_horizontal", 50.0)
        self.declare_parameter("sep_vertical", 20.0)
        self.declare_parameter("stale_timeout", 1.0)

        ids = list(self.get_parameter("uav_ids").value)
        prefix = str(self.get_parameter("odom_prefix").value)
        self.sep = Separation(
            float(self.get_parameter("sep_horizontal").value),
            float(self.get_parameter("sep_vertical").value))
        self.stale = float(self.get_parameter("stale_timeout").value)

        self.pos: Dict[str, tuple] = {}
        self.stamp: Dict[str, float] = {}
        self.violations = 0

        # 单机时订阅 <prefix>/odom；多机时按 <prefix>1/odom ... 逐机订阅
        if len(ids) == 1:
            topics = [("%s/odom" % prefix.rstrip("/"), ids[0])]
        else:
            topics = [("%s%d/odom" % (prefix.rstrip("/"), i + 1), uid)
                      for i, uid in enumerate(ids)]
        for topic, uid in topics:
            self.create_subscription(
                Odometry, topic,
                lambda msg, uid=uid: self._on_odom(uid, msg), 10)

        self.pub_status = self.create_publisher(String, "/uav_deconflict/status", 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/uav_deconflict/markers", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("间隔监视已启动，机队 %s，标准 %.0f/%.0f m"
                               % (",".join(ids), self.sep.horizontal,
                                  self.sep.vertical))

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, uid: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos[uid] = (p.x, p.y, p.z)
        self.stamp[uid] = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        # 判定逻辑在 core.monitor 里（无 ROS 依赖，有单元测试覆盖）
        fresh = fresh_positions(self.pos, self.stamp, now, self.stale)
        near: Closest = closest(fresh, self.sep)
        if near.violating:
            self.violations += 1

        self.pub_status.publish(String(
            data=json.dumps(near.as_dict(self.sep, self.violations),
                            ensure_ascii=False)))
        if near.violating:
            self.get_logger().warn("间隔侵入：%s↔%s 水平 %.1f m / 垂直 %.1f m"
                                   % (near.pair[0], near.pair[1],
                                      near.min_h or 0.0, near.min_v or 0.0))
        self._publish_markers(near, fresh)

    def _publish_markers(self, near: Closest, fresh: Dict[str, tuple]) -> None:
        arr = MarkerArray()
        if near.aircraft >= 2 and near.pair[0] in fresh and near.pair[1] in fresh:
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.35, 0.0, 1.0
            for uid in near.pair:
                p = fresh[uid]
                m.points.append(Point(x=p[0], y=p[1], z=p[2]))
            arr.markers.append(m)
        self.pub_markers.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeconflictMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
