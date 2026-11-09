#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端 RL 部署节点（ROS2/rclpy）。

加载 PPO 权重，订阅 /camera/image_raw 预测速度，发布 /uav/cmd_vel。
安全：订阅 /camera/depth，前方扇区最小深度小于 collision_depth 时直接急停，
不管模型输出什么。
第一帧图像到之前不预测（避免全黑图乱输出）。
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import e2e_logic as L


class DeployNode(Node):
    def __init__(self):
        super().__init__("e2e_rl_deploy")
        self.declare_parameter("model_path", "")
        self.declare_parameter("img_w", 64)
        self.declare_parameter("img_h", 48)
        self.declare_parameter("collision_depth", 0.6)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory("uav_e2e_rl"), "models", "ppo_e2e.zip")
        self.get_logger().info(f"loading model: {model_path}")
        from stable_baselines3 import PPO
        self.model = PPO.load(model_path)

        self._img = None
        self._depth = None
        self._cmd = self.create_publisher(Twist, "/uav/cmd_vel", 10)
        self.create_subscription(Image, "/camera/image_raw", self._img_cb, 10)
        self.create_subscription(Image, "/camera/depth", self._depth_cb, 10)
        self.timer = self.create_timer(0.1, self._step)

    def _img_cb(self, msg: Image):
        self._img = L.preprocess_image(
            msg.encoding, bytes(msg.data), msg.width, msg.height,
            int(self.get_parameter("img_w").value),
            int(self.get_parameter("img_h").value))

    def _depth_cb(self, msg: Image):
        h, w = msg.height, msg.width
        self._depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(h, w)

    def _publish_zero(self):
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = 0.0
        self._cmd.publish(tw)

    def _step(self):
        depth = self._depth
        if depth is not None and L.is_collision(
                depth, float(self.get_parameter("collision_depth").value)):
            self._publish_zero()
            self.get_logger().warn_throttle(1.0, "obstacle ahead, emergency stop")
            return

        if self._img is None:
            self.get_logger().warn_throttle(3.0, "waiting for first camera frame")
            return

        obs = self._img[np.newaxis, :, :].astype(np.float32)
        action, _ = self.model.predict(obs, deterministic=True)
        vx, wz = L.clip_action(float(action[0]), float(action[1]))
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        self._cmd.publish(tw)


def main():
    rclpy.init()
    node = DeployNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
