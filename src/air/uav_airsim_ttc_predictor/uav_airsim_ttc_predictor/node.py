"""ROS 2 节点：订阅前视深度，发布 TTC 预测。"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from .cnn import DepthPolicyNet

TTC_MAX = 20.0
SIZE = 64


class TtcNode(Node):
    def __init__(self):
        super().__init__("uav_ttc_predictor")
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_w = os.path.join(root, "models_trained", "ttc.npz")
        self.declare_parameter("weights", default_w)
        self.declare_parameter("ttc_max", TTC_MAX)
        self.ttc_max = float(self.get_parameter("ttc_max").value)

        self.net = DepthPolicyNet()
        self.net.load(self.get_parameter("weights").value)

        self.pub = self.create_publisher(Float32, "/uav/ttc", 10)
        self.create_subscription(Image, "/uav/depth", self.on_depth, 10)
        self.get_logger().info("TTC 预测节点就绪")

    def on_depth(self, msg: Image):
        # 期望 float32 单通道，按 30 m 归一化后缩放
        raw = np.frombuffer(msg.data, dtype=np.float32)
        n = int(np.sqrt(raw.size))
        arr = raw.reshape(n, n)
        idx = (np.linspace(0, n - 1, SIZE)).astype(int)
        small = np.clip(arr[np.ix_(idx, idx)] / 30.0, 0.0, 1.0)
        pred = float(self.net.forward(small.astype(np.float32))[0]) * self.ttc_max
        out = Float32()
        out.data = pred
        self.pub.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = TtcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()