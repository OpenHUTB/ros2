#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""时间偏移估计节点。

订阅两路带时间戳的角速度：

* ``imu_topic``：``sensor_msgs/msg/Imu``，取 ``angular_velocity``；
* ``cam_topic``：``geometry_msgs/msg/Vector3Stamped``，由视觉里程计 / VIO 前端
  给出的相机角速度。

在滑动窗口上做互相关，周期性把估计结果以 JSON 发布到 ``~/time_offset``。

约定：``offset_ms > 0`` 表示**相机时间戳滞后于 IMU**，即相机数据要往前推这么多
才与 IMU 对齐。
"""

from __future__ import annotations

import json
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from camera_imu_time_calibration import core


class TimeOffsetEstimator(Node):
    """在滑动窗口上估计相机相对 IMU 的常量时间偏移。"""

    def __init__(self):
        super().__init__("camera_imu_time_estimator")

        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("cam_topic", "/camera/angular_velocity")
        self.declare_parameter("window_sec", 8.0)
        self.declare_parameter("search_range_ms", 250.0)
        self.declare_parameter("min_samples", 40)
        self.declare_parameter("publish_period_sec", 1.0)

        self.window = float(self.get_parameter("window_sec").value)
        self.search_range = float(self.get_parameter("search_range_ms").value) / 1000.0
        self.min_samples = int(self.get_parameter("min_samples").value)
        period = float(self.get_parameter("publish_period_sec").value)

        self.imu_buf = deque()
        self.cam_buf = deque()

        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self.on_imu, 50)
        self.create_subscription(
            Vector3Stamped, str(self.get_parameter("cam_topic").value),
            self.on_cam, 50)
        self.pub = self.create_publisher(String, "~/time_offset", 10)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"窗口 {self.window:.1f} s，搜索范围 ±{self.search_range*1000:.0f} ms，"
            f"最少 {self.min_samples} 帧")

    # -- 回调 ---------------------------------------------------------------
    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_imu(self, msg: Imu) -> None:
        v = msg.angular_velocity
        self.imu_buf.append((self._stamp(msg), (v.x, v.y, v.z)))
        self._trim()

    def on_cam(self, msg: Vector3Stamped) -> None:
        v = msg.vector
        self.cam_buf.append((self._stamp(msg), (v.x, v.y, v.z)))
        self._trim()

    def _trim(self) -> None:
        for buf in (self.imu_buf, self.cam_buf):
            while buf and buf[0][0] < self._now() - self.window:
                buf.popleft()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- 估计 ---------------------------------------------------------------
    def on_timer(self) -> None:
        if len(self.cam_buf) < self.min_samples or len(self.imu_buf) < self.min_samples:
            self.get_logger().debug(
                f"样本不足：IMU {len(self.imu_buf)}，相机 {len(self.cam_buf)}")
            return

        t_imu = np.array([s for s, _ in self.imu_buf])
        w_imu = np.array([w for _, w in self.imu_buf])
        t_cam = np.array([s for s, _ in self.cam_buf])
        w_cam = np.array([w for _, w in self.cam_buf])

        if t_cam[-1] - t_cam[0] < 1.0:
            self.get_logger().debug("相机窗口不足 1 s，跳过")
            return

        res = core.estimate_offset(t_cam, w_cam, t_imu, w_imu,
                                   search_range=self.search_range)

        scale = res["scale"]
        payload = {
            "offset_ms": round(res["offset"] * 1000.0, 3),
            "correlation": round(res["correlation"], 5),
            "scale": None if math.isnan(scale) else round(scale, 4),
            "n_imu": int(len(t_imu)),
            "n_cam": int(len(t_cam)),
            "window_sec": round(float(t_cam[-1] - t_cam[0]), 2),
            "stamp": round(self._now(), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(
            f"offset = {payload['offset_ms']:+.2f} ms，"
            f"rho = {payload['correlation']:.4f}，"
            f"样本 IMU {payload['n_imu']} / 相机 {payload['n_cam']}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimeOffsetEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
