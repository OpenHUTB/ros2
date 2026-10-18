"""
vision_sim_node.py —— ROS2 节点：MuJoCo 仿真 + 前视图像发布
============================================================

在 ROS2 Humble 下运行（Ubuntu 20.04）。Windows 下仅做语法参考。
"""

from __future__ import annotations

import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from humanoid_vision_fall_prediction.sim import VisionFallSim, DEFAULT_STAND_TARGET

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "models", "humanoid_camera.xml")


class VisionSimNode(Node):
    def __init__(self):
        super().__init__("vision_sim_node")
        self.pub_img = self.create_publisher(Image, "~/front_image", 10)
        self.pub_ctrl = self.create_publisher(Float32MultiArray, "~/joint_cmd", 10)
        self.sub_ctrl = self.create_subscription(
            Float32MultiArray, "~/balance_cmd", self._on_balance_cmd, 10)
        self.target = DEFAULT_STAND_TARGET.copy()
        self.sim = VisionFallSim(MODEL_PATH)
        self.sim.reset()
        self.timer = self.create_timer(0.01, self._tick)
        self.get_logger().info("vision_sim_node started")

    def _on_balance_cmd(self, msg: Float32MultiArray):
        if len(msg.data) == 15:
            self.target = np.array(msg.data)

    def _tick(self):
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
