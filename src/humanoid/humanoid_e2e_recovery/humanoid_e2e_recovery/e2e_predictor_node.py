#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
e2e_predictor_node.py —— 端到端视觉平衡：MLP 推理节点
=====================================================
订阅：/front_camera/image  (sensor_msgs/msg/Image)
发布：/joint_targets        (std_msgs/msg/Float64MultiArray)
"""
from __future__ import annotations
import os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from humanoid_e2e_recovery.predictor import E2EPredictor


class E2EPredictorNode(Node):
    def __init__(self):
        super().__init__('e2e_predictor_node')

        self.declare_parameter('weights_path', '')
        weights = self.get_parameter('weights_path').value
        if not weights:
            weights = os.path.join(HERE, 'models_trained', 'e2e_policy.npz')

        self.predictor = E2EPredictor(weights)
        self.target_pub = self.create_publisher(Float64MultiArray, '/joint_targets', 10)

        self.create_subscription(Image, '/front_camera/image', self._img_cb, 10)
        self.get_logger().info(f'e2e_predictor_node 启动, weights={weights}')

    def _img_cb(self, msg: Image):
        # 解码 mono8 图像
        w, h = msg.width, msg.height
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        if arr.shape != (64, 64):
            return

        # MLP 推理
        target = self.predictor.predict_from_image(arr)

        out = Float64MultiArray()
        out.data = target.tolist()
        self.target_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = E2EPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
