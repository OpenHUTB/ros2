#!/usr/bin/env python3
"""RLW 预警节点。

纯 ROS 决策层：订阅桥接节点发布的交通灯状态、停止线距离与车速，
由固定周期定时器推进三级滞回状态机
（normal / approaching / violation_risk），发布分级状态与跃迁事件。
与 CARLA 完全解耦，可离线喂数据测试。

话题：
  订阅  /rlw/light_state  (std_msgs/String)   red / yellow / green / off
  订阅  /rlw/dist_to_stop (std_msgs/Float32)  自车到停止线距离（m）
  订阅  /rlw/ego_speed    (std_msgs/Float32)  自车速度（m/s）
  发布  /rlw/state  (std_msgs/String)  normal / approaching / violation_risk
  发布  /rlw/event  (std_msgs/String)  跃迁事件（仅跃迁拍发布一次）：
        approach_start / violation_start / violation_clear / approach_clear

实现说明：订阅回调只缓存最新数据，决策由固定周期定时器统一推进——
保证状态机每拍步进一次，不受消息到达顺序影响。
热身期（warmup_ticks）与 LDW 一致：冷启动灯态未稳定时不输出决策。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .red_light_core import RLWController, RLWParams


class RLWAlertNode(Node):

    def __init__(self):
        super().__init__("rlw_alert_node")
        self.declare_parameter("a_comfort", 2.5)
        self.declare_parameter("warn_enter", 30.0)
        self.declare_parameter("warn_exit", 35.0)
        self.declare_parameter("brake_margin", 1.2)
        self.declare_parameter("period", 0.05)
        self.declare_parameter("warmup_ticks", 2)

        self.p = RLWParams(
            a_comfort=float(self.get_parameter("a_comfort").value),
            warn_enter=float(self.get_parameter("warn_enter").value),
            warn_exit=float(self.get_parameter("warn_exit").value),
            brake_margin=float(self.get_parameter("brake_margin").value),
        )
        self.period = float(self.get_parameter("period").value)

        self.ctrl = RLWController(
            self.p,
            warmup_ticks=int(self.get_parameter("warmup_ticks").value))
        self._light = "off"
        self._dist = 1.0e4
        self._speed = 0.0
        self._last_state = self.ctrl._state

        self.sub_light = self.create_subscription(
            String, "rlw/light_state", self._on_light, 10)
        self.sub_dist = self.create_subscription(
            Float32, "rlw/dist_to_stop", self._on_dist, 10)
        self.sub_speed = self.create_subscription(
            Float32, "rlw/ego_speed", self._on_speed, 10)
        self.pub_state = self.create_publisher(String, "rlw/state", 10)
        self.pub_event = self.create_publisher(String, "rlw/event", 10)
        self.timer = self.create_timer(self.period, self._step)

    def _on_light(self, msg):
        if msg.data in ("red", "yellow", "green", "off"):
            self._light = msg.data

    def _on_dist(self, msg):
        self._dist = float(msg.data)

    def _on_speed(self, msg):
        self._speed = float(msg.data)

    def _step(self):
        """一个预警拍：状态机推进 → 发布状态/事件。"""
        r = self.ctrl.tick(self._light, self._dist, self._speed, self.period)
        self.pub_state.publish(String(data=r["state"]))
        if r["event"] and r["state"] != self._last_state:
            self.pub_event.publish(String(data=r["event"]))
            self._last_state = r["state"]
            self.get_logger().info(
                f"状态迁移 → {r['state']}（灯 {r['light']}，"
                f"停止线 {r['dist']:.1f} m，车速 {r['speed']:.1f} m/s）")


def main(args=None):
    rclpy.init(args=args)
    node = RLWAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
