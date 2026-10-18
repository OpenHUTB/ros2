"""策略推理节点：加载 TorchScript 策略，把 40 维观测映射为 12 维**残差动作**。

残差由 ``sim_node`` 叠加到平衡基元输出上（见 :mod:`.balance`）::

    q_target = ComBalance(...) + RESIDUAL_SCALE * a

因此本节点只负责神经网络推理，不含任何控制逻辑，可单独替换。
"""

from __future__ import annotations

import os

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .g1_env import ACT_DIM, OBS_DIM


class PolicyNode(Node):
    def __init__(self):
        super().__init__("humanoid_policy_node")

        self.declare_parameter("weights", "weights/policy_g1.pt")
        self.declare_parameter("action_clip", 1.0)

        weights = self.get_parameter("weights").value
        if not os.path.exists(weights):
            raise FileNotFoundError(
                f"未找到策略权重：{weights}。请先运行 scripts/export_policy.py。"
            )
        self.policy = torch.jit.load(weights, map_location="cpu").eval()
        self.clip = float(self.get_parameter("action_clip").value)

        self.pub = self.create_publisher(Float64MultiArray, "humanoid/joint_targets", 10)
        self.create_subscription(Float64MultiArray, "humanoid/obs", self._on_obs, 10)

        self.infer_count = 0
        self.get_logger().info(f"策略节点已启动：权重 {weights}")

    def _on_obs(self, msg: Float64MultiArray) -> None:
        obs = np.asarray(msg.data, dtype=np.float32)
        if obs.size != OBS_DIM:
            self.get_logger().warn(
                f"观测维度不符：收到 {obs.size}，期望 {OBS_DIM}", throttle_duration_sec=2.0
            )
            return
        with torch.no_grad():
            action = self.policy(torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()
        action = np.clip(action, -self.clip, self.clip)

        out = Float64MultiArray()
        out.data = action.astype(float).tolist()
        self.pub.publish(out)

        self.infer_count += 1
        if self.infer_count % 250 == 0:
            self.get_logger().info(
                f"已推理 {self.infer_count} 帧，动作范数 {float(np.linalg.norm(action)):.3f}"
            )


def main(argv=None):
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
