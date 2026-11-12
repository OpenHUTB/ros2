#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""部署节点：前视图像 -> CNN 推理 -> /uav/cmd_vel（ROS2/rclpy）。

安全：
- 权重缺失/加载失败立即安全停止，不创建发布者；
- 模型输出 NaN/inf/长度异常时丢弃本帧；
- 输出经 sanitize_action 限幅后下发。
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import preprocess as P
from .model import build_model
from .safety import (
    load_trained_model,
    sanitize_action,
    WeightMissingError,
    WeightLoadError,
)


class _RosLogger:
    def __init__(self, node):
        self._n = node

    def info(self, msg):
        self._n.get_logger().info(msg)

    def warn(self, msg):
        self._n.get_logger().warn(msg)

    def error(self, msg):
        self._n.get_logger().error(msg)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("uav_neural_perception")
        self.declare_parameter("model_path", "")
        self.declare_parameter("max_vx", 2.0)
        self.declare_parameter("max_vy", 0.5)
        self.declare_parameter("max_yaw", 0.4)
        self.declare_parameter("gain_vx", 1.0)
        self.declare_parameter("gain_vy", 1.0)
        self.declare_parameter("gain_yaw", 1.0)
        self.declare_parameter("cmd_vel_topic", "/uav/cmd_vel")

        import torch
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory("uav_neural_perception"),
                "models", "best_drone_model.pth")

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_yaw = float(self.get_parameter("max_yaw").value)
        self.gain = (float(self.get_parameter("gain_vx").value),
                     float(self.get_parameter("gain_vy").value),
                     float(self.get_parameter("gain_yaw").value))

        try:
            self.model = load_trained_model(
                model_path, self.device, build_model,
                allow_random=False, log=_RosLogger(self))
        except (WeightMissingError, WeightLoadError) as exc:
            self.get_logger().fatal(f"[安全停止] {exc}")
            raise SystemExit(1)

        self._pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        self.get_logger().info("神经网络感知节点已启动，等待 /camera/image_raw ...")

    def _on_image(self, msg: Image):
        try:
            img_rgb = P.decode_image(msg.encoding, bytes(msg.data),
                                     msg.width, msg.height)
        except ValueError as e:
            self.get_logger().warn_throttle(2.0, f"无法解码图像: {e}")
            return

        tensor = P.preprocess_image(img_rgb)[np.newaxis, ...]
        import torch
        with torch.no_grad():
            out = self.model(torch.from_numpy(tensor).to(self.device))[0].cpu().numpy()

        action = P.denormalize_action(out)
        action = [action[0] * self.gain[0],
                  action[1] * self.gain[1],
                  action[2] * self.gain[2]]
        result = sanitize_action(action, self.max_vx, self.max_vy, self.max_yaw)
        if result is None:
            self.get_logger().warn_throttle(
                1.0, "模型输出非法（NaN/inf/格式异常），已丢弃本帧")
            return
        vx, vy, yaw = result
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = vx, vy, 0.0
        tw.angular.z = yaw
        self._pub.publish(tw)
        self.get_logger().info_throttle(
            1.0, f"下发: vx={vx:.2f} vy={vy:.2f} yaw={yaw:.2f}")


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
