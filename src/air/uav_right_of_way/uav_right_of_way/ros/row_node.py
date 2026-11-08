#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线通行权决策节点：订阅多机里程计，触发时发布让路指令与决策理由。

参数（``config/uav_right_of_way.yaml``）::

    uav_ids: ["UAV-1", ...]
    odom_prefix: "/uav"          # 话题名为 <prefix><n>/odom
    horizon_s: 8.0
    cpa_threshold_m: 40.0
    decision_interval_s: 0.2
    clear_hold_s: 3.0

发布::

    /uav_right_of_way/commands   std_msgs/String
        JSON：{"stamp", "decisions": [{uid, other, rule, action, reason, ...}]}

判定逻辑（威胁检测、规则引擎、理由字符串）全部在 ``core`` 里并有单元
测试覆盖；本节点只是薄壳：订阅 odom → 组装 ``AircraftState`` → 每个决策
周期调一次 ``detect`` + ``rules`` → 发布 JSON 并写日志。让路指令的执行
（改参考航向/高度/速度）由各机的飞行控制侧完成，本节点不动航迹。
"""
from __future__ import annotations

import json
import math
from typing import Dict, List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_right_of_way.core.detect import (AircraftState, DetectParams, Threat,
                                          predict_threat)
from uav_right_of_way.core.rules import RuleParams, decide


class RightOfWayNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_right_of_way_node")
        self.declare_parameter("uav_ids", ["UAV-1", "UAV-2"])
        self.declare_parameter("odom_prefix", "/uav")
        self.declare_parameter("horizon_s", 8.0)
        self.declare_parameter("cpa_threshold_m", 40.0)
        self.declare_parameter("decision_interval_s", 0.2)
        self.declare_parameter("clear_hold_s", 3.0)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("turn_right_deg", 30.0)
        self.declare_parameter("descend_m", 10.0)
        self.declare_parameter("slow_fraction", 0.30)
        self.declare_parameter("overtake_lateral_m", 20.0)

        ids = [str(v) for v in self.get_parameter("uav_ids").value]
        prefix = str(self.get_parameter("odom_prefix").value).rstrip("/")
        self.dp = DetectParams(
            horizon_s=float(self.get_parameter("horizon_s").value),
            cpa_threshold_m=float(self.get_parameter("cpa_threshold_m").value))
        self.rp = RuleParams(
            turn_right_deg=float(self.get_parameter("turn_right_deg").value),
            descend_m=float(self.get_parameter("descend_m").value),
            slow_fraction=float(self.get_parameter("slow_fraction").value),
            overtake_lateral_m=float(
                self.get_parameter("overtake_lateral_m").value))
        self.decide_dt = float(self.get_parameter("decision_interval_s").value)
        self.clear_hold = float(self.get_parameter("clear_hold_s").value)
        self.stale = float(self.get_parameter("stale_timeout").value)

        self.states: Dict[str, AircraftState] = {}
        self.stamp: Dict[str, float] = {}
        self.last_decide = -math.inf
        # 机对 episode：{frozenset({a,b}): 最近确认威胁时刻}
        self.episodes: Dict[frozenset, float] = {}

        if len(ids) == 1:
            topics = [("%s/odom" % prefix, ids[0])]
        else:
            topics = [("%s%d/odom" % (prefix, i + 1), uid)
                      for i, uid in enumerate(ids)]
        for topic, uid in topics:
            self.create_subscription(
                Odometry, topic,
                lambda msg, uid=uid: self._on_odom(uid, msg), 10)

        self.pub_cmd = self.create_publisher(
            String, "/uav_right_of_way/commands", 10)
        self.create_timer(max(self.decide_dt, 0.05), self._tick)
        self.get_logger().info(
            "通行权决策已启动，机队 %s，预测 %.0f s 内 CPA < %.0f m 触发"
            % (",".join(ids), self.dp.horizon_s, self.dp.cpa_threshold_m))

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, uid: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        prio = 0
        try:
            prio = int(msg.child_frame_id.split("_")[-1]) \
                if msg.child_frame_id.startswith("prio_") else 0
        except Exception:
            prio = 0
        self.states[uid] = AircraftState(
            uid, [p.x, p.y, p.z], [v.x, v.y, v.z], prio)
        self.stamp[uid] = self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 周期
    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = {uid: st for uid, st in self.states.items()
                 if now - self.stamp.get(uid, -math.inf) <= self.stale}
        if len(fresh) < 2:
            return
        if now - self.last_decide < self.decide_dt - 1e-9:
            return
        self.last_decide = now

        uids = sorted(fresh)
        decisions: List[dict] = []
        for i, a in enumerate(uids):
            for b in uids[i + 1:]:
                key = frozenset((a, b))
                own, other = fresh[a], fresh[b]
                th = predict_threat(own, other, self.dp)
                if th is None:
                    # 威胁消失且超过观察期 → 结束该机对 episode
                    if key in self.episodes and \
                            now - self.episodes[key] >= self.clear_hold:
                        del self.episodes[key]
                    continue
                # CPA 对双方对称；每个机对 episode 只产出一次决策
                is_new = key not in self.episodes
                self.episodes[key] = now
                if is_new:
                    decisions.append(
                        decide(own, other, th, now, self.rp).as_dict())
                    decisions.append(
                        decide(other, own,
                               Threat(own.uid, th.cpa_t, th.cpa_dist),
                               now, self.rp).as_dict())

        if decisions:
            payload = dict(stamp=now, decisions=decisions)
            self.pub_cmd.publish(String(data=json.dumps(
                payload, ensure_ascii=False)))
            for d in decisions:
                self.get_logger().info(
                    "让路指令 %s ← %s [%s] %s：%s"
                    % (d["uid"], d["other"], d["rule"],
                       d["action"]["kind"], d["reason"]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RightOfWayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
