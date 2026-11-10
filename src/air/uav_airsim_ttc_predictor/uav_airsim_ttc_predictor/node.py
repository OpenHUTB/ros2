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

        # load() 是 classmethod，返回新的已加载实例；必须接住返回值
        self.net = DepthPolicyNet.load(self.get_parameter("weights").value)
        self.get_logger().info("已加载权重 %s" % self.get_parameter("weights").value)

        self.pub = self.create_publisher(Float32, "/uav/ttc", 10)
        self.create_subscription(Image, "/uav/depth", self.on_depth, 10)
        self.get_logger().info("TTC 预测节点就绪")

    def on_depth(self, msg: Image):
        # 期望 float32 单通道，按 30 m 归一化后缩放到 64x64。
        # 尺寸以 msg.height / msg.width 为准：AirSim DepthPlanar 默认 640x480，
        # 不是正方形，用 sqrt(msg.data.size) 会得到 554 并直接 ValueError。
        raw = np.frombuffer(msg.data, dtype=np.float32)
        h = int(msg.height)
        w = int(msg.width)
        if h <= 0 or w <= 0 or h * w != raw.size:
            # 消息头没填尺寸时退化成单列，至少不会崩
            self.get_logger().warn("深度图尺寸与消息头不符，按 1 x %d 处理" % raw.size)
            h, w = 1, int(raw.size)
        arr = raw.reshape(h, w)
        idx_h = np.linspace(0, h - 1, SIZE).astype(int)
        idx_w = np.linspace(0, w - 1, SIZE).astype(int)
        small = np.clip(arr[np.ix_(idx_h, idx_w)] / 30.0, 0.0, 1.0)
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