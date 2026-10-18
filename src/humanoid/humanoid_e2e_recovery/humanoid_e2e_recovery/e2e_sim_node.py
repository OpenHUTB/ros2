#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
e2e_sim_node.py —— 端到端视觉平衡：MuJoCo 仿真节点
===================================================
发布：/front_camera/image  (sensor_msgs/msg/Image)
订阅：/joint_targets       (std_msgs/msg/Float64MultiArray)
"""
from __future__ import annotations
import os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

# 支持直接 python3 运行（Windows 开发用）
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from humanoid_e2e_recovery.sim import E2ESim, DEFAULT_STAND_TARGET


class E2ESimNode(Node):
    def __init__(self):
        super().__init__('e2e_sim_node')

        # 参数
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').value
        if not model_path:
            model_path = os.path.join(HERE, 'models', 'humanoid_camera.xml')

        self.sim = E2ESim(model_path)
        self.sim.reset()

        # 发布图像
        self.img_pub = self.create_publisher(Image, '/front_camera/image', 10)
        self.latest_target = DEFAULT_STAND_TARGET.copy()

        # 订阅关节角目标
        self.create_subscription(
            Float64MultiArray, '/joint_targets', self._target_cb, 10)

        # 定时器：500Hz 步仿真
        self.timer = self.create_timer(0.002, self._step)
        self.get_logger().info(f'e2e_sim_node 启动, model={model_path}')

    def _target_cb(self, msg: Float64MultiArray):
        arr = np.array(msg.data, dtype=np.float64)
        if arr.shape[0] == 15:
            self.latest_target = arr

    def _step(self):
        # 走几步仿真
        for _ in range(4):
            self.sim.step(target=self.latest_target)

        # 发布图像（灰度 64x64 → 1 通道）
        gray = self.sim.render_front(gray=True)  # shape=(64,64)
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.height, img_msg.width = gray.shape
        img_msg.encoding = 'mono8'
        img_msg.step = gray.shape[1]
        img_msg.data = gray.astype(np.uint8).tobytes()
        self.img_pub.publish(img_msg)

    def destroy_node(self):
        self.sim.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = E2ESimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
