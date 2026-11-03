#!/usr/bin/env python3
"""BSD 预警决策节点。

纯 ROS 决策层：订阅桥接节点发布的左右盲区距离与自车速度，
运行 BSD 滞回状态机，发布分级预警。与 CARLA 完全解耦，
可离线喂数据测试。

话题：
  订阅  /bsd/left_distance  /bsd/right_distance (std_msgs/Float32)
  发布  /bsd/alert  (std_msgs/String)   JSON：{"left": "normal|approach|warn", ...}
  发布  /bsd/event  (std_msgs/String)   跃迁事件（left_warn / right_warn / ...）

预警分级（每侧独立）：
  normal   盲区无目标或目标远离
  approach 目标尚在盲区外但正快速接近（变道前提前提醒）
  warn     目标已进入盲区（禁止/谨慎变道）

实现说明：订阅回调只缓存最新距离，决策由固定周期定时器统一推进——
保证左右两侧状态机每拍同步步进，不受消息到达顺序影响。
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .bsd_core import BSDController, BSDParams, WARN


class BSDAlertNode(Node):

    def __init__(self):
        super().__init__("bsd_alert_node")
        self.declare_parameter("warn_enter", 12.0)
        self.declare_parameter("warn_exit", 15.0)
        self.declare_parameter("approach_enter", 25.0)
        self.declare_parameter("close_min", 0.5)
        self.declare_parameter("valid_max", 40.0)
        self.declare_parameter("period", 0.05)

        p = BSDParams(
            warn_enter=float(self.get_parameter("warn_enter").value),
            warn_exit=float(self.get_parameter("warn_exit").value),
            approach_enter=float(self.get_parameter("approach_enter").value),
            close_min=float(self.get_parameter("close_min").value),
            valid_max=float(self.get_parameter("valid_max").value),
        )
        self.ctrl = BSDController(p)
        self.valid_max = p.valid_max

        self._d = {"left": p.valid_max, "right": p.valid_max}  # 最新缓存
        self._levels = {"left": "normal", "right": "normal"}   # 上一拍分级

        self.sub_left = self.create_subscription(
            Float32, "bsd/left_distance", lambda m: self._on("left", m), 10)
        self.sub_right = self.create_subscription(
            Float32, "bsd/right_distance", lambda m: self._on("right", m), 10)
        self.pub_alert = self.create_publisher(String, "bsd/alert", 10)
        self.pub_event = self.create_publisher(String, "bsd/event", 10)
        self.timer = self.create_timer(
            float(self.get_parameter("period").value), self._step)

    def _on(self, side, msg):
        self._d[side] = float(msg.data)

    def _level(self, r):
        """单侧分级：warn > approach > normal。"""
        if r["state"] == WARN:
            return "warn"
        if r["approach"]:
            return "approach"
        return "normal"

    def _step(self):
        """一个决策拍：推进状态机并发布分级/事件。"""
        results = self.ctrl.tick(self._d["left"], self._d["right"])
        events = []
        for side, r in results.items():
            level = self._level(r)
            if level != self._levels[side]:
                if level == "warn":
                    events.append(f"{side}_warn")
                elif level == "normal" and self._levels[side] != "normal":
                    events.append(f"{side}_clear")
                self._levels[side] = level
            results[side]["level"] = level
        self.pub_alert.publish(String(data=json.dumps(
            {s: r["level"] for s, r in results.items()})))
        if events:
            self.pub_event.publish(String(data=",".join(events)))
            self.get_logger().info(
                f"盲区事件: {events}  d_left={results['left']['distance']:.1f} "
                f"d_right={results['right']['distance']:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = BSDAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
