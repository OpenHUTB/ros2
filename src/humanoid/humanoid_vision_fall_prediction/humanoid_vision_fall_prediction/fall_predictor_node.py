"""
fall_predictor_node.py —— ROS2 节点：MLP 推理 + 平衡控制
=========================================================

订阅 /front_image，运行 FallPredictor，根据预测类别发布 /balance_cmd。
"""

from __future__ import annotations

import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from ament_index_python.packages import get_package_share_directory

from humanoid_vision_fall_prediction.predictor import FallPredictor
from humanoid_vision_fall_prediction.controller import BalanceController


def _weights_path() -> str:
    """优先从 colcon install 的 share/models_trained 找权重；否则回退源码目录。"""
    try:
        share = get_package_share_directory("humanoid_vision_fall_prediction")
        p = os.path.join(share, "models_trained", "fall_predictor.npz")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "..", "models_trained", "fall_predictor.npz")


class FallPredictorNode(Node):
    def __init__(self):
        super().__init__("fall_predictor_node")
        self.predictor = FallPredictor(_weights_path())
        self.ctrl = BalanceController()
        # 全局话题，与 vision_sim_node 直接对接
        self.pub = self.create_publisher(Float32MultiArray, "balance_cmd", 10)
        self.sub = self.create_subscription(
            Image, "front_image", self._on_image, 10)
        self.get_logger().info(
            f"fall_predictor_node started, weights={_weights_path()}")

    def _on_image(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        img = img.astype(np.float32) / 255.0
        label, cls, conf = self.predictor.predict_from_image(img)
        target = self.ctrl.target_for(cls, conf)
        out = Float32MultiArray()
        out.data = target.tolist()
        self.pub.publish(out)
        self.get_logger().debug(f"predict={label} conf={conf:.2f}")


def main():
    rclpy.init()
    node = FallPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
