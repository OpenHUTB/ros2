#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""演示用合成数据源：没有实车/实机时也能把整条链路跑起来。

以节点时钟为时间基准，发布两路数据：

* ``imu_topic``（``sensor_msgs/msg/Imu``，默认 200 Hz）：真值 + 常值零偏 + 噪声；
* ``cam_topic``（``geometry_msgs/msg/Vector3Stamped``，默认 30 Hz）：
  第 k 条消息的时间戳是当前时刻 ``t``，内容却是 ``omega(t - offset)``，
  也就是把 ``offset`` 毫秒的滞后"做进"数据里，供估计节点找回来。

注意：本节点不模拟相机帧内积分，只制造时间偏移。离线仿真
（``core.simulate``）里才有帧平均模型。
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time as RosTime
from sensor_msgs.msg import Imu

from camera_imu_time_calibration import core
from camera_imu_time_calibration.ros_params import (
    integer_param,
    number_param,
    shutdown_quietly,
    string_param,
)


class SyntheticSource(Node):
    """按真值角速度发布 IMU 与"带偏移的相机角速度"。"""

    def __init__(self):
        super().__init__("camera_imu_synthetic_source")

        imu_topic = string_param(self, "imu_topic", "/imu")
        cam_topic = string_param(self, "cam_topic", "/camera/angular_velocity")
        imu_rate = number_param(self, "imu_rate", 200.0)
        cam_rate = number_param(self, "cam_rate", 30.0)
        self.offset = number_param(self, "offset_ms", 50.0) / 1000.0
        self.bias = number_param(self, "imu_bias", 0.005)
        self.imu_noise = number_param(self, "imu_noise", 0.003)
        self.cam_noise = number_param(self, "cam_noise", 0.010)
        self.rng = np.random.default_rng(integer_param(self, "seed", 0))

        self.imu_pub = self.create_publisher(Imu, imu_topic, 50)
        self.cam_pub = self.create_publisher(Vector3Stamped, cam_topic, 50)

        self.t0 = self._now()
        self.create_timer(1.0 / imu_rate, self.on_imu)
        self.create_timer(1.0 / cam_rate, self.on_cam)

        self.get_logger().info(
            f"IMU {imu_rate:.0f} Hz → {imu_topic}，"
            f"相机 {cam_rate:.0f} Hz → {cam_topic}，"
            f"注入偏移 {self.offset*1000:+.1f} ms")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self, t: float):
        """把秒数转成 ``builtin_interfaces/Time`` 消息。"""
        return RosTime(seconds=t).to_msg()

    def on_imu(self) -> None:
        t = self._now()
        w = core.omega(t - self.t0) + self.bias
        w = w + self.rng.normal(0.0, self.imu_noise, 3)

        msg = Imu()
        msg.header.stamp = self._stamp(t)
        msg.header.frame_id = "imu_link"
        msg.angular_velocity.x = float(w[0])
        msg.angular_velocity.y = float(w[1])
        msg.angular_velocity.z = float(w[2])
        self.imu_pub.publish(msg)

    def on_cam(self) -> None:
        t = self._now()
        # 内容滞后 offset；时间戳仍然是当前时刻 —— 这就是要估计的偏移
        w = core.omega(t - self.t0 - self.offset)
        w = w + self.rng.normal(0.0, self.cam_noise, 3)

        msg = Vector3Stamped()
        msg.header.stamp = self._stamp(t)
        msg.header.frame_id = "camera_link"
        msg.vector.x = float(w[0])
        msg.vector.y = float(w[1])
        msg.vector.z = float(w[2])
        self.cam_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown_quietly()


if __name__ == "__main__":
    main()
