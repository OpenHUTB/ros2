#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
e2e_sim_node.py —— 端到端视觉平衡：MuJoCo 仿真节点
===================================================
发布：/front_camera/image  (sensor_msgs/msg/Image, mono8)
订阅：/joint_targets       (std_msgs/msg/Float64MultiArray, 15 维关节目标角)
"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from ament_index_python.packages import get_package_share_directory

from humanoid_e2e_recovery.sim import E2ESim, DEFAULT_STAND_TARGET

# 模型步长 0.002s；ROS 定时器 0.01s → 每 tick 步进 5 次，保持实时
SIM_STEPS_PER_TICK = 5


def _model_path() -> str:
    try:
        share = get_package_share_directory("humanoid_e2e_recovery")
        p = os.path.join(share, "models", "humanoid_camera.xml")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "..", "models", "humanoid_camera.xml")


class E2ESimNode(Node):
    def __init__(self):
        super().__init__('e2e_sim_node')

        model_path = self.declare_parameter('model_path', '').value
        if not model_path:
            model_path = _model_path()

        self.sim = E2ESim(model_path)
        self.sim.reset()

        self.img_pub = self.create_publisher(Image, 'front_camera/image', 10)
        self.latest_target = DEFAULT_STAND_TARGET.copy()

        self.create_subscription(
            Float64MultiArray, 'joint_targets', self._target_cb, 10)

        self.timer = self.create_timer(0.01, self._step)
        self.get_logger().info(
            f'e2e_sim_node 启动, model={model_path}, '
            f'steps_per_tick={SIM_STEPS_PER_TICK}')

    def _target_cb(self, msg: Float64MultiArray):
        arr = np.array(msg.data, dtype=np.float64)
        if arr.shape[0] == 15:
            self.latest_target = arr

    def _step(self):
        for _ in range(SIM_STEPS_PER_TICK):
            self.sim.step(target=self.latest_target)

        gray = self.sim.render_front(gray=True)  # shape=(64,64), [0,1] float
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.height, img_msg.width = gray.shape
        img_msg.encoding = 'mono8'
        img_msg.step = gray.shape[1]
        # [0,1] float → [0,255] uint8（之前漏乘 255，导致全黑）
        img_msg.data = (gray * 255.0).astype(np.uint8).tobytes()
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
