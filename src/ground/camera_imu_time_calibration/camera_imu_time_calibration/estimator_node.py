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

除了单次估计值，节点还输出**重估稳定性**：最近 ``stability_window`` 次重估的
标准差（``recent_std_ms``）。它是对"常量偏移假设是否成立"的经验检验——若估计值
随时间线性漂移，说明时钟频差不可忽略，此时应把模型升级成 ``offset(t) = a + b·t``。
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
from camera_imu_time_calibration.ros_params import (
    integer_param,
    number_param,
    shutdown_quietly,
    string_param,
)


class TimeOffsetEstimator(Node):
    """在滑动窗口上估计相机相对 IMU 的常量时间偏移。"""

    def __init__(self):
        super().__init__("camera_imu_time_estimator")

        imu_topic = string_param(self, "imu_topic", "/imu")
        cam_topic = string_param(self, "cam_topic", "/camera/angular_velocity")
        self.window = number_param(self, "window_sec", 8.0)
        self.search_range = number_param(self, "search_range_ms", 250.0) / 1000.0
        self.min_samples = integer_param(self, "min_samples", 40)
        period = number_param(self, "publish_period_sec", 1.0)

        # 重估稳定性：最近 N 次估计的离散度，用作"常量假设是否成立"的经验判据
        self.stability_n = max(2, integer_param(self, "stability_window", 10))
        self.stability_warn_ms = number_param(self, "stability_warn_ms", 2.0)
        self.recent_offsets = deque(maxlen=self.stability_n)
        self._unstable = False

        self.imu_buf = deque()
        self.cam_buf = deque()

        self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.create_subscription(Vector3Stamped, cam_topic, self.on_cam, 50)
        self.pub = self.create_publisher(String, "~/time_offset", 10)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"IMU {imu_topic} / 相机 {cam_topic}；"
            f"窗口 {self.window:.1f} s，搜索范围 ±{self.search_range*1000:.0f} ms，"
            f"最少 {self.min_samples} 帧；"
            f"稳定性看最近 {self.stability_n} 次，阈值 {self.stability_warn_ms:.1f} ms")

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
        """按**消息时间戳**裁窗，不用节点时钟。

        缓冲区以 ``header.stamp`` 为键，而时钟基准未必与节点时钟一致（设备
        时钟、独立的 ``use_sim_time`` 来源、rosbag 回放的时钟）。拿
        ``self._now()`` 去比消息时间戳，窗口要么永不裁（缓冲无限增长），
        要么一裁到底。具体逻辑在 ``core.trim_buffers``，纯函数、可单测。
        """

        core.trim_buffers((self.imu_buf, self.cam_buf), self.window)

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

        offset_ms = res["offset"] * 1000.0
        self.recent_offsets.append(offset_ms)
        std_ms = (float(np.std(self.recent_offsets))
                  if len(self.recent_offsets) >= 2 else None)
        stable = None if std_ms is None else bool(std_ms <= self.stability_warn_ms)

        scale = res["scale"]
        payload = {
            "offset_ms": round(offset_ms, 3),
            "correlation": round(res["correlation"], 5),
            "scale": None if math.isnan(scale) else round(scale, 4),
            "n_imu": int(len(t_imu)),
            "n_cam": int(len(t_cam)),
            "window_sec": round(float(t_cam[-1] - t_cam[0]), 2),
            # 重估稳定性：最近 recent_n 次 offset_ms 的标准差
            "recent_std_ms": None if std_ms is None else round(std_ms, 3),
            "recent_n": int(len(self.recent_offsets)),
            "stable": stable,
            "stamp": round(self._now(), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

        std_txt = "—" if std_ms is None else f"{std_ms:.2f}"
        self.get_logger().info(
            f"offset = {payload['offset_ms']:+.2f} ms，"
            f"rho = {payload['correlation']:.4f}，"
            f"重估 std = {std_txt} ms，"
            f"样本 IMU {payload['n_imu']} / 相机 {payload['n_cam']}")

        # 只在"稳定 → 不稳定"的时刻告警一次，避免 1 Hz 刷屏
        if std_ms is not None and std_ms > self.stability_warn_ms:
            if not self._unstable:
                self.get_logger().warn(
                    f"最近 {len(self.recent_offsets)} 次重估的 std = {std_ms:.2f} ms，"
                    f"超过阈值 {self.stability_warn_ms:.2f} ms：常量偏移假设可能不成立"
                    f"（时钟频差 / 时变延迟），或运动激励不足。"
                    f"可先看 correlation 是否 < 0.9，再决定是否把模型升级为 offset(t)=a+b*t。")
                self._unstable = True
        elif std_ms is not None:
            if self._unstable:
                self.get_logger().info(f"重估已稳定：std = {std_ms:.2f} ms")
            self._unstable = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimeOffsetEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown_quietly()


if __name__ == "__main__":
    main()
