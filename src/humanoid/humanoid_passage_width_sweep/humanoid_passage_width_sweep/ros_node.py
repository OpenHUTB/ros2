# -*- coding: utf-8 -*-
"""ROS 2 节点：把 PassageSim 跑成一条可观测的「实验循环」。

话题
----
* ``~/state``   ``std_msgs/Float64MultiArray`` = [x, y, yaw, v, w, clearance, contact, width]
* ``~/result``  ``std_msgs/Float64MultiArray`` = [reached(0/1), 用时, 最小净空, 路径长度]
* ``~/reset``   ``std_msgs/Empty``   收到后立即重置到起点

节点刻意做得很薄：场景、规划、指标都在不依赖 rclpy 的模块里，
所以没有 ROS 也能用 ``python3 main.py --test`` 完整验证。
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray

import humanoid_passage_width_sweep.config as config
import humanoid_passage_width_sweep.planners as planners
from humanoid_passage_width_sweep.sim import PassageSim


class PassageSweepNode(Node):
    """窄通道通行实验节点。"""

    def __init__(self) -> None:
        super().__init__("passage_sweep_node")
        self.declare_parameter("width", config.DEFAULT_WIDTH)
        self.declare_parameter("variant", "mppi_footprint")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("loop", True)

        self.width = float(self.get_parameter("width").value)
        self.variant = str(self.get_parameter("variant").value)
        self.rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.loop = bool(self.get_parameter("loop").value)

        self.sim = PassageSim(width=self.width, variant=self.variant)
        self._min_clear = math.inf
        self._path = 0.0
        self._prev = None

        self.pub_state = self.create_publisher(Float64MultiArray, "~/state", 10)
        self.pub_result = self.create_publisher(Float64MultiArray, "~/result", 10)
        self.create_subscription(Empty, "~/reset", self._on_reset, 10)
        self.create_timer(1.0 / self.rate, self._on_timer)

        self.get_logger().info(
            f"窄通道通行启动：门宽 {self.width:.2f} m，变体 {self.variant}"
            f"（{planners.label_of(self.variant)}），{self.rate:.0f} Hz")

    def _reset(self) -> None:
        self.sim.reset()
        self._min_clear = math.inf
        self._path = 0.0
        self._prev = None

    def _on_reset(self, _msg: Empty) -> None:
        self._reset()
        self.get_logger().info("已重置到起点")

    def _on_timer(self) -> None:
        s = self.sim.step()
        x, y = s["x"], s["y"]
        if self._prev is not None:
            self._path += math.hypot(x - self._prev[0], y - self._prev[1])
        self._prev = (x, y)
        self._min_clear = min(self._min_clear, s["clearance"])

        st = Float64MultiArray()
        st.data = [x, y, s["yaw"], s["v"], s["w"], s["clearance"], s["contact"], self.width]
        self.pub_state.publish(st)

        reached = self.sim.goal_distance() <= config.GOAL_RADIUS
        if reached or s["contact"] > 0.5:
            res = Float64MultiArray()
            res.data = [1.0 if (reached and not self.sim.failed) else 0.0,
                        self.sim.time, self._min_clear, self._path]
            self.pub_result.publish(res)
            self.get_logger().info(
                f"{'到达目标' if reached and not self.sim.failed else '碰撞/失败'}："
                f"用时 {self.sim.time:.1f} s，最小净空 {self._min_clear:+.3f} m")
            if self.loop:
                self._reset()

    def destroy_node(self):
        self.sim.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PassageSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()