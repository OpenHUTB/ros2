#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
e2e_predictor_node.py —— 端到端视觉平衡：MLP 推理节点
=====================================================
订阅：front_camera/image  (sensor_msgs/msg/Image, mono8)
发布：joint_targets        (std_msgs/msg/Float64MultiArray, 15 维)
"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from ament_index_python.packages import get_package_share_directory

from humanoid_e2e_recovery.predictor import E2EPredictor


def _weights_path() -> str:
    try:
        share = get_package_share_directory("humanoid_e2e_recovery")
        p = os.path.join(share, "models_trained", "e2e_policy.npz")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "..", "models_trained", "e2e_policy.npz")


class E2EPredictorNode(Node):
    def __init__(self):
        super().__init__('e2e_predictor_node')

        weights = self.declare_parameter('weights_path', '').value
        if not weights:
            weights = _weights_path()

        self.predictor = E2EPredictor(weights)
        self.target_pub = self.create_publisher(Float64MultiArray, 'joint_targets', 10)

        self.create_subscription(Image, 'front_camera/image', self._img_cb, 10)
        self.get_logger().info(f'e2e_predictor_node 启动, weights={weights}')

    def _img_cb(self, msg: Image):
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        if arr.shape != (64, 64):
            return
        # uint8 [0,255] → float [0,1]，与训练数据一致
        img_float = arr.astype(np.float32) / 255.0
        target = self.predictor.predict_from_image(img_float)

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
