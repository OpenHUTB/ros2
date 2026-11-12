"""ROS 2 nodes: gyro/visual-rate replay and the offset estimator."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String

from .core import estimate_offset, residual_rms
from .scene import SignalConfig, simulate_pair


class VioSimulator(Node):
    """Publish gyro and visual-rate streams with a known delay."""

    def __init__(self) -> None:
        super().__init__("vio_sim")
        self.declare_parameter("rate_hz", 200.0)
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("offset_s", 0.025)
        self.declare_parameter("seed", 0)
        self.config = SignalConfig(
            rate_hz=float(self.get_parameter("rate_hz").value),
            duration_s=float(self.get_parameter("duration_s").value),
            offset_s=float(self.get_parameter("offset_s").value),
            seed=int(self.get_parameter("seed").value),
        )
        self.pair = simulate_pair(self.config)
        self.gyro_pub = self.create_publisher(Imu, "/vio/gyro", 50)
        self.visual_pub = self.create_publisher(Float64, "/vio/visual_rate", 50)
        self.index = 0
        self.create_timer(1.0 / self.config.rate_hz, self.step)
        self.get_logger().info(
            f"vio_sim ready: {self.config.samples} samples, true offset "
            f"{self.config.offset_s * 1000.0:.0f} ms"
        )

    def step(self) -> None:
        index = self.index % self.config.samples
        message = Imu()
        message.header.frame_id = "imu_link"
        message.angular_velocity.z = float(self.pair["gyro"][index])
        self.gyro_pub.publish(message)
        visual = Float64()
        visual.data = float(self.pair["visual"][index])
        self.visual_pub.publish(visual)
        self.index += 1


class TimeOffsetEstimator(Node):
    """Buffer both streams and estimate the offset once enough data arrived."""

    def __init__(self) -> None:
        super().__init__("time_offset_estimator")
        self.declare_parameter("min_samples", 400)
        self.declare_parameter("dt", 0.005)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.dt = float(self.get_parameter("dt").value)
        self.gyro: list[float] = []
        self.visual: list[float] = []
        self.visual_buffer: list[float] = []
        self.create_subscription(Imu, "/vio/gyro", self.on_gyro, 100)
        self.create_subscription(Float64, "/vio/visual_rate", self.on_visual, 100)
        self.pub = self.create_publisher(String, "/vio/time_offset", 10)
        self.done = False
        self.get_logger().info(f"time_offset_estimator ready: need {self.min_samples} samples")

    def on_visual(self, message: Float64) -> None:
        self.visual_buffer.append(float(message.data))

    def on_gyro(self, message: Imu) -> None:
        self.gyro.append(float(message.angular_velocity.z))
        if self.visual_buffer:
            self.visual.append(self.visual_buffer.pop(0))
        if self.done or len(self.gyro) < self.min_samples:
            return
        estimate = estimate_offset(np.asarray(self.gyro), np.asarray(self.visual), self.dt)
        payload = estimate.to_dict()
        payload["residual_before"] = residual_rms(
            np.asarray(self.gyro), np.asarray(self.visual), 0.0, self.dt
        )
        payload["residual_after"] = residual_rms(
            np.asarray(self.gyro), np.asarray(self.visual), estimate.offset_s, self.dt
        )
        self.pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"offset {payload['offset_s'] * 1000.0:.2f} ms, peak {payload['peak_correlation']:.3f}"
        )
        self.done = True


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_vio_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(VioSimulator())


def main_time_offset_estimator(args=None) -> None:
    rclpy.init(args=args)
    _spin(TimeOffsetEstimator())
