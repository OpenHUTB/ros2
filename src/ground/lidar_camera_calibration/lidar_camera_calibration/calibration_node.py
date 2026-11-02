#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机-激光雷达外参标定 ROS 2 节点。

职责
----
1. 订阅 ``sensor_msgs/CameraInfo``（默认 ``/camera_info``）获取相机内参；
   若超时未收到，则使用内置默认内参并给出警告（离线演示也能跑）。
2. 用内置合成场景做一次外参标定，把结果发布到
   ``/lidar_camera_calibration/extrinsics``（``std_msgs/String``，JSON 形式）。

说明：本节点不订阅真实点云/图像；合成数据的真值验证见离线脚本。
接入真实传感器的数据采集与棋盘格检测列在文档"未验证项"。

运行::

    source /opt/ros/humble/setup.bash
    python3 calibration_node.py
    # 或
    ros2 launch lidar_camera_calibration main.launch.py

手动喂一个 CameraInfo 观察内参覆盖::

    ros2 topic pub -1 /camera_info sensor_msgs/msg/CameraInfo \\
        "{width: 640, height: 480, k: [700.0, 0.0, 320.0, 0.0, 700.0, 240.0, 0.0, 0.0, 1.0], d: [0.0, 0.0, 0.0, 0.0, 0.0]}"
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

try:  # 作为包导入（colcon / -m）
    from .calibration_core import DEFAULT_DIST, DEFAULT_K, calibrate, matrix_to_text
except ImportError:  # 直接以脚本方式运行
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from calibration_core import DEFAULT_DIST, DEFAULT_K, calibrate, matrix_to_text


class LidarCameraCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_camera_calibration')

        self.declare_parameter('views', 3)
        self.declare_parameter('pixel_noise', 1.0)
        self.declare_parameter('range_noise', 0.01)
        self.declare_parameter('angle_noise_deg', 0.1)
        self.declare_parameter('seed', 0)
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_info_timeout', 3.0)
        self.declare_parameter('use_camera_info', True)

        self._K = DEFAULT_K.copy()
        self._D = DEFAULT_DIST.copy()
        self._got_info = False
        self._done = False
        self._t0 = self.get_clock().now()
        self._result_msg = None

        self._pub = self.create_publisher(
            String, '/lidar_camera_calibration/extrinsics', 10)
        # 标定只做一次，但结果按 1 Hz 重发：ROS 话题不是持久化的，
        # 晚启动的订阅者（例如手动 echo）否则会完全收不到结果。
        self.create_timer(1.0, self._republish)

        if bool(self.get_parameter('use_camera_info').value):
            topic = str(self.get_parameter('camera_info_topic').value)
            self.create_subscription(CameraInfo, topic, self._on_camera_info, 10)
            self.get_logger().info(
                '等待相机内参: %s（超时 %.1f s 后使用内置默认内参）'
                % (topic, float(self.get_parameter('camera_info_timeout').value)))
        else:
            self.get_logger().info('use_camera_info=false，直接使用内置默认内参')

        self.create_timer(0.2, self._maybe_run)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._done or self._got_info:
            return
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            self.get_logger().warn('收到无效 CameraInfo（焦距 <= 0），忽略')
            return
        self._K = np.array(
            [[msg.k[0], 0.0, msg.k[2]],
             [0.0, msg.k[4], msg.k[5]],
             [0.0, 0.0, 1.0]], dtype=np.float64)
        self._D = np.asarray(msg.d[:5], dtype=np.float64) if msg.d else DEFAULT_DIST
        self._got_info = True
        self.get_logger().info(
            '已接收相机内参: fx=%.1f fy=%.1f cx=%.1f cy=%.1f'
            % (msg.k[0], msg.k[4], msg.k[2], msg.k[5]))

    def _maybe_run(self) -> None:
        if self._done:
            return
        timeout = float(self.get_parameter('camera_info_timeout').value)
        elapsed = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        if not self._got_info and elapsed < timeout:
            return
        if not self._got_info:
            self.get_logger().warn(
                '未在 %.1f s 内收到 CameraInfo，使用内置默认内参（离线演示模式）'
                % timeout)
        self._run_once()

    def _run_once(self) -> None:
        self._done = True
        views = int(self.get_parameter('views').value)
        try:
            res = calibrate(
                views=views,
                pixel_noise=float(self.get_parameter('pixel_noise').value),
                range_noise=float(self.get_parameter('range_noise').value),
                angle_noise_deg=float(self.get_parameter('angle_noise_deg').value),
                seed=int(self.get_parameter('seed').value),
                K=self._K, D=self._D)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('标定失败: %s' % exc)
            return

        self.get_logger().info(
            '标定完成: 旋转误差 %.3f°，平移误差 %.4f m，重投影 RMSE %.3f px'
            % (res.rot_error_deg, res.trans_error_m, res.rmse_px))
        self.get_logger().info('估计外参: %s' % matrix_to_text(res.R_est, res.t_est))

        payload = {
            'extrinsics': matrix_to_text(res.R_est, res.t_est),
            'rot_error_deg': round(res.rot_error_deg, 4),
            'trans_error_m': round(res.trans_error_m, 5),
            'rmse_px': round(res.rmse_px, 4),
            'views': res.views,
            'n_points': res.n_points,
            'camera_info_used': self._got_info,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._result_msg = msg
        self._pub.publish(msg)

    def _republish(self) -> None:
        """按 1 Hz 重发最近一次标定结果，保证晚到的订阅者也能收到。"""
        if self._result_msg is not None:
            self._pub.publish(self._result_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarCameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
