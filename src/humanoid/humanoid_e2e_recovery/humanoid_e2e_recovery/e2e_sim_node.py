#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
e2e_sim_node.py —— 端到端视觉平衡：MuJoCo 仿真节点
===================================================
发布：/front_camera/image  (sensor_msgs/msg/Image, mono8)
订阅：/torque_delta      (std_msgs/msg/Float64MultiArray, 15 维关节力矩增量 N·m)
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
        self.latest_tau = np.zeros(15)

        self.create_subscription(
            Float64MultiArray, 'torque_delta', self._tau_cb, 10)

        self.timer = self.create_timer(0.01, self._step)
        self.get_logger().info(
            f'e2e_sim_node 启动, model={model_path}, '
            f'steps_per_tick={SIM_STEPS_PER_TICK}')

        # 闭环可观测性：每 1 s 打一行站姿状态，跌倒只报一次 ERROR。
        # 没有这条日志，ROS 侧无法证明"站住了"，只能看图像猜。
        self.t_sim = 0.0
        self._next_log_t = 0.0
        self._fall_reported = False
        self._max_tilt = 0.0

    def _tau_cb(self, msg: Float64MultiArray):
        arr = np.array(msg.data, dtype=np.float64)
        if arr.shape[0] == 15:
            self.latest_tau = arr

    def _step(self):
        for _ in range(SIM_STEPS_PER_TICK):
            self.sim.step(target=DEFAULT_STAND_TARGET, torque_extra=self.latest_tau)
        self.t_sim += SIM_STEPS_PER_TICK * self.sim.model.opt.timestep

        tilt = abs(np.degrees(self.sim.torso_tilt()))
        self._max_tilt = max(self._max_tilt, tilt)

        if self.t_sim >= self._next_log_t:
            self._next_log_t += 1.0
            self.get_logger().info(
                f't={self.t_sim:6.2f}s root_z={self.sim.root_z:.3f}m '
                f'tilt={tilt:.1f}deg max_tilt={self._max_tilt:.1f}deg '
                f'ankle_tau={float(self.latest_tau[3]):+.1f}N*m crashed={self.sim.fallen()}')

        if self.sim.fallen() and not self._fall_reported:
            self._fall_reported = True
            self.get_logger().error(
                f'跌倒：t={self.t_sim:.2f}s root_z={self.sim.root_z:.3f}m '
                f'tilt={tilt:.1f}deg（判据 root_z<0.6）')

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
