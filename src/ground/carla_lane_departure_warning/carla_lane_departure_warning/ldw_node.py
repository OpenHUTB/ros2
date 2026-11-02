#!/usr/bin/env python3
"""LDW 预警节点。

纯 ROS 决策层：订阅桥接节点发布的带符号横向偏移，
由固定周期定时器推进三级滞回状态机（normal/deviation/departure），
发布分级状态与跃迁事件。与 CARLA 完全解耦，可离线喂数据测试。

话题：
  订阅  /ldw/lateral_offset  (std_msgs/Float32)  带符号横向偏移，左偏正
  发布  /ldw/state  (std_msgs/String)   normal / deviation / departure
  发布  /ldw/event  (std_msgs/String)   跃迁事件（仅跃迁拍发布一次）：
        deviation_start_left / deviation_start_right /
        departure_start_left / departure_start_right /
        deviation_clear / departure_end

实现说明：订阅回调只缓存最新偏移，决策由固定周期定时器统一推进——
保证状态机每拍步进一次，不受消息到达顺序影响。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .lane_departure_core import LDWController, LDWParams


class LDWAlertNode(Node):

    def __init__(self):
        super().__init__("ldw_alert_node")
        self.declare_parameter("warn_enter", 0.5)
        self.declare_parameter("warn_exit", 0.3)
        self.declare_parameter("depart_enter", 0.9)
        self.declare_parameter("depart_exit", 0.65)
        self.declare_parameter("period", 0.05)

        self.p = LDWParams(
            warn_enter=float(self.get_parameter("warn_enter").value),
            warn_exit=float(self.get_parameter("warn_exit").value),
            depart_enter=float(self.get_parameter("depart_enter").value),
            depart_exit=float(self.get_parameter("depart_exit").value),
        )
        self.period = float(self.get_parameter("period").value)

        self.ctrl = LDWController(self.p)
        self._offset = 0.0
        self._last_state = self.ctrl._state

        self.sub_offset = self.create_subscription(
            Float32, "ldw/lateral_offset", self._on_offset, 10)
        self.pub_state = self.create_publisher(String, "ldw/state", 10)
        self.pub_event = self.create_publisher(String, "ldw/event", 10)
        self.timer = self.create_timer(self.period, self._step)

    def _on_offset(self, msg):
        self._offset = float(msg.data)

    def _step(self):
        """一个预警拍：状态机推进 → 发布状态/事件。"""
        r = self.ctrl.tick(self._offset, self.period)
        self.pub_state.publish(String(data=r["state"]))
        if r["event"] and r["state"] != self._last_state:
            self.pub_event.publish(String(data=r["event"]))
            self._last_state = r["state"]
            self.get_logger().info(
                f"状态迁移 → {r['state']}（偏移 {r['offset']:+.2f} m）")


def main(args=None):
    rclpy.init(args=args)
    node = LDWAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
