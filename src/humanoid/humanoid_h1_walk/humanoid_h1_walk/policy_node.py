"""策略推理节点：把 41/47 维观测映射成 10/12 维关节动作。

本节点只做神经网络前向（TorchScript，CPU 单线程约 0.1 ms），不含物理与 PD 控制，
因此可以和仿真节点分别部署、分别替换：换成别的策略只要保持观测/动作维度不变。

动作不是最终关节角，仿真节点按下式缩放后再交给 PD 控制器::

    q_target = a · action_scale + q_default
"""

from __future__ import annotations

import os
import time

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .walker import load_specs, load_torchscript


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("humanoid_h1_walk_policy")

        self.declare_parameter("robot", "h1")
        self.declare_parameter("config", "")
        self.declare_parameter("weights", "")

        robot = str(self.get_parameter("robot").value)
        config = str(self.get_parameter("config").value) or None
        specs = load_specs(config)
        if robot not in specs:
            raise KeyError(f"未知机型 {robot}，可选：{sorted(specs)}")
        self.spec = specs[robot]

        weights = str(self.get_parameter("weights").value) or str(self.spec.weights)
        if not os.path.exists(weights):
            raise FileNotFoundError(f"未找到策略权重：{weights}")
        self.policy = load_torchscript(weights)
        self.policy.eval()
        torch.set_num_threads(1)

        self.pub = self.create_publisher(Float64MultiArray, "humanoid/joint_targets", 10)
        self.create_subscription(Float64MultiArray, "humanoid/obs", self._on_obs, 10)

        self.count = 0
        self.elapsed = 0.0
        self.get_logger().info(
            f"{robot} 策略节点已启动：{weights}，观测 {self.spec.num_obs} 维 → "
            f"动作 {self.spec.num_actions} 维"
        )

    def _on_obs(self, msg: Float64MultiArray) -> None:
        obs = np.asarray(msg.data, dtype=np.float32)
        if obs.size != self.spec.num_obs:
            self.get_logger().warn(
                f"观测维度不符：收到 {obs.size}，期望 {self.spec.num_obs}", throttle_duration_sec=2.0
            )
            return

        t0 = time.perf_counter()
        with torch.no_grad():
            action = self.policy(torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()
        self.elapsed += time.perf_counter() - t0

        out = Float64MultiArray()
        out.data = action.astype(float).tolist()
        self.pub.publish(out)

        self.count += 1
        if self.count % 250 == 0:
            self.get_logger().info(
                f"已推理 {self.count} 帧，平均单帧 {self.elapsed / self.count * 1e3:.2f} ms，"
                f"动作范数 {float(np.linalg.norm(action)):.3f}"
            )


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
