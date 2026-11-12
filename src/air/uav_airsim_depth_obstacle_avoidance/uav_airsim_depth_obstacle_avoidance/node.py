"""ROS 2 节点：把训练好的深度避障策略接到话题上。

节点本身不直接连 AirSim，只做「深度图进、控制量出」的推理，
这样在有 ROS 2 的机器上可以接任何发布深度图的仿真或真机驱动。

    /uav/depth     sensor_msgs/Image      深度图（64x64 直接用；其他分辨率按消息头缩放）
    /uav/cmd       std_msgs/Float32MultiArray  [yaw_rate(rad/s), z_target(m)]
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

from .cnn import DepthPolicyNet
from .policy import ALT_DELTA_MAX, YAW_RATE_MAX

SIZE = 64


class DepthAvoidNode(Node):
    def __init__(self):
        super().__init__("uav_depth_avoidance")

        default_weights = os.path.join(
            get_package_share_directory("uav_airsim_depth_obstacle_avoidance"),
            "models_trained", "depth_avoid.npz")
        self.declare_parameter("weights", default_weights)
        self.declare_parameter("z_ref", -5.0)

        # load() 是 classmethod，返回新的已加载实例；必须接住返回值，
        # 否则 self.net 仍是随机初始化权重，发布的是噪声。
        self.net = DepthPolicyNet.load(self.get_parameter("weights").value)
        self.z_ref = float(self.get_parameter("z_ref").value)

        self.pub = self.create_publisher(Float32MultiArray, "/uav/cmd", 10)
        self.sub = self.create_subscription(Image, "/uav/depth", self.on_depth, 10)
        self.get_logger().info("深度避障节点已启动，权重 %s" % self.get_parameter("weights").value)

    def on_depth(self, msg: Image):
        # 尺寸以 msg.height / msg.width 为准。网络第一层全连接固定为 8*15*15，
        # 只接受 64x64；收到 AirSim 原始的 640x480 时先缩放，不能假设是方图。
        raw = np.frombuffer(msg.data, dtype=np.float32)
        h = int(msg.height)
        w = int(msg.width)
        if h <= 0 or w <= 0 or h * w != raw.size:
            self.get_logger().warn("深度图尺寸与消息头不符，按 1 x %d 处理" % raw.size)
            h, w = 1, int(raw.size)
        depth = raw.reshape(h, w)

        if depth.max() > 1.5:          # 传进来的是米制深度，先归一化
            depth = np.clip(depth, 0.0, 20.0) / 20.0

        if (h, w) != (SIZE, SIZE):
            idx_h = np.linspace(0, h - 1, SIZE).astype(int)
            idx_w = np.linspace(0, w - 1, SIZE).astype(int)
            depth = depth[np.ix_(idx_h, idx_w)]

        yaw_norm, alt_norm = self.net.predict(depth)
        yaw_rate = float(yaw_norm) * YAW_RATE_MAX
        z_target = float(np.clip(self.z_ref - float(alt_norm) * ALT_DELTA_MAX, -9.0, -3.0))

        out = Float32MultiArray()
        out.data = [yaw_rate, z_target]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthAvoidNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
