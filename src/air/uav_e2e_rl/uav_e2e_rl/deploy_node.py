#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端 RL 部署节点（ROS2/rclpy）。

加载 PPO 权重，订阅 /camera/image_raw，发布 /uav/cmd_vel。
第一帧图像到之前不预测（避免全黑图乱输出）。
"""
from __future__ import annotations

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from . import e2e_logic as L


class DeployNode(Node):
    def __init__(self):
        super().__init__("e2e_rl_deploy")
        self.declare_parameter("model_path", "")
        self.declare_parameter("img_w", 64)
        self.declare_parameter("img_h", 48)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            share = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                                 "share", "uav_e2e_rl", "models", "ppo_e2e.zip")
            model_path = os.path.normpath(share)
        self.get_logger().info(f"loading model: {model_path}")
        from stable_baselines3 import PPO
        self.model = PPO.load(model_path)

        self._img = None
        self._cmd = self.create_publisher(Twist, "/uav/cmd_vel", 10)
        self.create_subscription(Image, "/camera/image_raw", self._img_cb, 10)
        self.timer = self.create_timer(0.1, self._step)

    def _img_cb(self, msg: Image):
        self._img = L.preprocess_image(
            msg.encoding, bytes(msg.data), msg.width, msg.height,
            int(self.get_parameter("img_w").value),
            int(self.get_parameter("img_h").value))

    def _step(self):
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
