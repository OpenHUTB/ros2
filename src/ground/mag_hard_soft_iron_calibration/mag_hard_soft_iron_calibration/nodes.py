"""ROS 2 nodes: magnetometer replay and an online calibration node."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String

from .core import fit_ellipsoid, magnetic_heading
from .scene import MagConfig, simulate_calibration


class MagSimulator(Node):
    """Publish calibration-sample measurements as ``sensor_msgs/MagneticField``."""

    def __init__(self) -> None:
        super().__init__("mag_sim")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("interval_s", 1.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.interval_s = max(0.0, float(self.get_parameter("interval_s").value))
        self.config = MagConfig(seed=int(self.get_parameter("seed").value))
        self.data = simulate_calibration(self.config)
        self.publisher = self.create_publisher(MagneticField, "/mag/data", 20)
        self.imu_publisher = self.create_publisher(Imu, "/mag/attitude", 20)
        self.index = 0
        self.create_timer(1.0 / self.rate_hz, self.step)
        self.get_logger().info(
            f"mag_sim ready: {len(self.data['measured_ut'])} samples, "
            f"{self.config.field_norm_ut:.0f} uT field"
        )

    def step(self) -> int:
        index = self.index % len(self.data["measured_ut"])
        field = self.data["measured_ut"][index]
        message = MagneticField()
        message.header.frame_id = "mag_link"
        message.header.stamp = self.get_clock().now().to_msg()
        message.magnetic_field.x = float(field[0])
        message.magnetic_field.y = float(field[1])
        message.magnetic_field.z = float(field[2])
        self.publisher.publish(message)
        attitude = Imu()
        attitude.header.frame_id = "mag_link"
        roll = float(self.data["roll_rad"][index])
        pitch = float(self.data["pitch_rad"][index])
        attitude.linear_acceleration.x = float(-9.81 * np.sin(pitch))
        attitude.linear_acceleration.y = float(9.81 * np.sin(roll) * np.cos(pitch))
        attitude.linear_acceleration.z = float(9.81 * np.cos(roll) * np.cos(pitch))
        self.imu_publisher.publish(attitude)
        self.index += 1
        return index


class MagCalibrator(Node):
    """Accumulate samples, fit the ellipsoid and publish the correction matrix."""

    def __init__(self) -> None:
        super().__init__("mag_calibrator")
        self.declare_parameter("samples", 600)
        self.declare_parameter("field_norm_ut", 48.0)
        self.samples_required = max(20, int(self.get_parameter("samples").value))
        self.field_norm = float(self.get_parameter("field_norm_ut").value)
        self.samples: list[list[float]] = []
        self.gravity: list[list[float]] = []
        self.create_subscription(MagneticField, "/mag/data", self.on_field, 50)
        self.create_subscription(Imu, "/mag/attitude", self.on_attitude, 50)
        self.status_pub = self.create_publisher(String, "/mag/status", 10)
        self.done = False
        self.get_logger().info(f"mag_calibrator ready: waiting for {self.samples_required} samples")

    def on_attitude(self, message: Imu) -> None:
        self.gravity.append(
            [
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ]
        )

    def on_field(self, message: MagneticField) -> None:
        if self.done:
            return
        self.samples.append(
            [
                float(message.magnetic_field.x),
                float(message.magnetic_field.y),
                float(message.magnetic_field.z),
            ]
        )
        if len(self.samples) < self.samples_required:
            return
        data = np.asarray(self.samples, dtype=float)
        try:
            fitted = fit_ellipsoid(data, self.field_norm)
        except ValueError as error:
            self.get_logger().warning(f"calibration failed: {error}")
            return
        heading = None
        if len(self.gravity) >= 1:
            heading = float(
                magnetic_heading(fitted.apply(data[-1:]), np.asarray(self.gravity[-1:]))[0]
            )
        payload = {
            "offset": fitted.offset.tolist(),
            "matrix": fitted.matrix.tolist(),
            "field_norm": fitted.field_norm,
            "residual_rms": fitted.residual_rms,
            "heading_rad": heading,
            "samples": fitted.samples,
        }
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"calibration done: residual {fitted.residual_rms:.2f} uT, "
            f"offset {np.round(fitted.offset, 2).tolist()}"
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


def main_mag_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(MagSimulator())


def main_mag_calibrator(args=None) -> None:
    rclpy.init(args=args)
    _spin(MagCalibrator())
