# -*- coding: utf-8 -*-
"""ROS 2 节点：把 :class:`HopperSim` 包成一个可 launch 的节点。

话题
----
* 订阅 ``~/target``      ``std_msgs/Float64MultiArray`` = [目标顶点高度, 目标前进速度]
* 发布 ``~/state``       ``sensor_msgs/JointState``   关节位置/速度（root_x, root_z, hip, leg）
* 发布 ``~/status``      ``std_msgs/Float64MultiArray`` = [z, vz, vx, q, 注能力, 相位(0飞1撑), 辨识效率]

设计上刻意保持很薄：控制律与仿真都在 ``controller.py`` / ``sim.py`` 里，
不依赖 rclpy，因此可以在没有 ROS 的环境下用 ``python3 main.py --test`` 完整验证。
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import mujoco_hopper_slip_control.config as config
from mujoco_hopper_slip_control.sim import HopperSim

JOINT_NAMES = ["root_x", "root_z", "hip", "leg"]


class HopperSlipNode(Node):
    """单足弹跳机器人 ROS 2 节点。"""

    def __init__(self) -> None:
        super().__init__("hopper_slip_node")

        self.declare_parameter("target_height", config.TARGET_HEIGHT)
        self.declare_parameter("target_speed", config.TARGET_SPEED)
        self.declare_parameter("control_hz", 100.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("start_q", config.START_Q)

        self.target_height = float(self.get_parameter("target_height").value)
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.control_hz = max(10.0, float(self.get_parameter("control_hz").value))
        self.publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))

        self.sim = HopperSim(target_height=self.target_height,
                             target_speed=self.target_speed,
                             start_q=float(self.get_parameter("start_q").value))

        # 每个定时器周期推进多少个物理步
        self._steps_per_cycle = max(1, int(round(
            (1.0 / self.control_hz) / config.TIMESTEP)))

        self.pub_state = self.create_publisher(JointState, "~/state", 10)
        self.pub_status = self.create_publisher(Float64MultiArray, "~/status", 10)
        self.sub_target = self.create_subscription(
            Float64MultiArray, "~/target", self._on_target, 10)

        self.create_timer(1.0 / self.control_hz, self._on_control)
        self.create_timer(1.0 / self.publish_rate, self._on_publish)

        self.get_logger().info(
            f"单足弹跳已启动：目标高度 {self.target_height:.2f} m，"
            f"目标速度 {self.target_speed:.2f} m/s，控制 {self.control_hz:.0f} Hz"
            f"（每周期 {self._steps_per_cycle} 个物理步）")

    # ------------------------------------------------------------------ 回调
    def _on_target(self, msg: Float64MultiArray) -> None:
        """接收 [目标高度, 目标速度]，缺项保持原值。"""
        data: List[float] = list(msg.data)
        if len(data) >= 1 and math.isfinite(data[0]):
            self.target_height = float(min(max(data[0], 0.35), 0.90))
            self.sim.controller.target_height = self.target_height
        if len(data) >= 2 and math.isfinite(data[1]):
            self.target_speed = float(min(max(data[1], -0.4), 0.4))
            self.sim.controller.target_speed = self.target_speed
        self.get_logger().info(
            f"新目标：高度 {self.target_height:.2f} m，速度 {self.target_speed:.2f} m/s")

    def _on_control(self) -> None:
        for _ in range(self._steps_per_cycle):
            self.sim.step()

    def _on_publish(self) -> None:
        s = self.sim.state()
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = "world"
        js.name = JOINT_NAMES
        js.position = [s["x"], s["z"], s["hip_q"], s["q"]]
        js.velocity = [s["vx"], s["vz"], 0.0, s["qd"]]
        self.pub_state.publish(js)

        st = Float64MultiArray()
        st.data = [s["z"], s["vz"], s["vx"], s["q"], self.sim.controller.f_ex,
                   1.0 if self.sim.controller.phase.value == "stance" else 0.0,
                   self.sim.controller.state.efficiency]
        self.pub_status.publish(st)

    def destroy_node(self) -> bool:
        self.sim.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HopperSlipNode()
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
