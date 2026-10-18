"""
vision_sim_node.py —— ROS2 节点：MuJoCo 仿真 + 前视图像发布
============================================================

在 ROS2 Humble 下运行（Ubuntu 20.04）。Windows 下仅做语法参考。

话题（全局，便于跨节点直接对接）：
  发布 /front_image   (sensor_msgs/Image, mono8)
  发布 /joint_cmd     (std_msgs/Float32MultiArray, 15 维)
  订阅 /balance_cmd   (std_msgs/Float32MultiArray, 15 维)
"""

from __future__ import annotations

import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from ament_index_python.packages import get_package_share_directory

from humanoid_vision_fall_prediction.sim import VisionFallSim, DEFAULT_STAND_TARGET

# 模型步长 0.002s，ROS 定时器 0.01s → 每个 tick 步进 5 次，保持实时性
SIM_STEPS_PER_TICK = 5


def _model_path() -> str:
    """优先从 colcon install 的 share 目录找模型；否则回退到源码目录。"""
    try:
        share = get_package_share_directory("humanoid_vision_fall_prediction")
        p = os.path.join(share, "models", "humanoid_camera.xml")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "..", "models", "humanoid_camera.xml")


class VisionSimNode(Node):
    def __init__(self):
        super().__init__("vision_sim_node")
        # 全局话题（无前导 ~），保证与 fall_predictor_node 直接对接
        self.pub_img = self.create_publisher(Image, "front_image", 10)
        self.pub_ctrl = self.create_publisher(Float32MultiArray, "joint_cmd", 10)
        self.sub_ctrl = self.create_subscription(
            Float32MultiArray, "balance_cmd", self._on_balance_cmd, 10)
        self.target = DEFAULT_STAND_TARGET.copy()
        self.sim = VisionFallSim(_model_path())
        self.sim.reset()
        # 每个 tick 步进 SIM_STEPS_PER_TICK 次，匹配 0.002s 物理步长
        self.timer = self.create_timer(0.01, self._tick)
        self.get_logger().info(
            f"vision_sim_node started, model={_model_path()}, "
            f"steps_per_tick={SIM_STEPS_PER_TICK}")

    def _on_balance_cmd(self, msg: Float32MultiArray):
        if len(msg.data) == 15:
            self.target = np.array(msg.data)

    def _tick(self):
        for _ in range(SIM_STEPS_PER_TICK):
            self.sim.step(target=self.target)
        img = self.sim.render_front(gray=True)
        msg = Image()
        msg.height, msg.width = img.shape
        msg.encoding = "mono8"
        msg.data = (img * 255).astype(np.uint8).tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_img.publish(msg)
        ctrl = Float32MultiArray()
        ctrl.data = self.target.tolist()
        self.pub_ctrl.publish(ctrl)


def main():
    rclpy.init()
    node = VisionSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
