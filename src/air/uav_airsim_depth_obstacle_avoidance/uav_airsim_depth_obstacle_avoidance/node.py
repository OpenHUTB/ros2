"""ROS 2 节点：把训练好的深度避障策略接到话题上。

节点本身不直接连 AirSim，只做「深度图进、控制量出」的推理，
这样在有 ROS 2 的机器上可以接任何发布深度图的仿真或真机驱动。

    /uav/depth     sensor_msgs/Image      64x64 float32 归一化深度
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


class DepthAvoidNode(Node):
    def __init__(self):
        super().__init__("uav_depth_avoidance")

        default_weights = os.path.join(
            get_package_share_directory("uav_airsim_depth_obstacle_avoidance"),
            "models_trained", "depth_avoid.npz")
        self.declare_parameter("weights", default_weights)
        self.declare_parameter("z_ref", -5.0)

        self.net = DepthPolicyNet()
        self.net.load(self.get_parameter("weights").value)
        self.z_ref = float(self.get_parameter("z_ref").value)

        self.pub = self.create_publisher(Float32MultiArray, "/uav/cmd", 10)
        self.sub = self.create_subscription(Image, "/uav/depth", self.on_depth, 10)
        self.get_logger().info("深度避障节点已启动，权重 %s" % self.get_parameter("weights").value)

    def on_depth(self, msg: Image):
        depth = np.array(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        if depth.max() > 1.5:          # 传进来的是米制深度，先归一化
            depth = np.clip(depth, 0.0, 20.0) / 20.0

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
