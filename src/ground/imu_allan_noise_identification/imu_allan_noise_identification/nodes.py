"""ROS 2 nodes: an IMU replay source and an online Allan-variance identifier."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .core import allan_deviation, fit_noise_model
from .scene import ImuConfig, simulate_tilt_sweep


def _quaternion_from_roll(roll: float):
    return float(np.sin(roll / 2.0)), float(np.cos(roll / 2.0))


class ImuSimulator(Node):
    """Replay the deterministic tilt sweep as a ``sensor_msgs/Imu`` stream."""

    def __init__(self) -> None:
        super().__init__("imu_sim")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("duration_s", 0.0)
        self.declare_parameter("seed", 0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.seed = int(self.get_parameter("seed").value)
        config = ImuConfig(
            rate_hz=self.rate_hz,
            sweep_duration_s=max(300.0, self.duration_s),
            seed=self.seed,
        )
        self.sweep = simulate_tilt_sweep(config)
        self.publisher = self.create_publisher(Imu, "/imu/data", 20)
        self.epoch = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)
        self.get_logger().info(
            f"imu_sim ready: {len(self.sweep['time_s'])} samples at {self.rate_hz:.0f} Hz"
        )

    def step(self) -> None:
        index = self.epoch
        if index >= len(self.sweep["time_s"]):
            if self.duration_s > 0.0:
                self.get_logger().info("replay finished")
                rclpy.shutdown()
            else:
                self.epoch = 0
            return
        message = Imu()
        message.header.frame_id = "imu_link"
        message.header.stamp = self.get_clock().now().to_msg()
        message.angular_velocity.z = float(self.sweep["gyro_deg_s"][index])
        quaternion = _quaternion_from_roll(float(np.deg2rad(self.sweep["tilt_deg"][index])))
        message.orientation.x = quaternion[0]
        message.orientation.w = quaternion[1]
        message.linear_acceleration.y = float(np.deg2rad(self.sweep["accel_tilt_deg"][index]))
        message.linear_acceleration.z = 1.0
        self.publisher.publish(message)
        self.epoch += 1


class AllanIdentifier(Node):
    """Collect a static gyro log, then identify ARW and bias instability."""

    def __init__(self) -> None:
        super().__init__("allan_identifier")
        self.declare_parameter("max_samples", 36000)
        self.declare_parameter("cluster_points", 20)
        self.declare_parameter("output_csv", "/tmp/allan_identifier.csv")
        self.max_samples = int(self.get_parameter("max_samples").value)
        self.cluster_points = int(self.get_parameter("cluster_points").value)
        self.samples: list[float] = []
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file, lineterminator="\n")
        self.writer.writerow(["index", "gyro_z", "tilt_rad"])
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)
        self.status_pub = self.create_publisher(String, "/imu/allan", 10)
        self.get_logger().info(
            f"allan_identifier ready: waiting for {self.max_samples} samples"
        )

    def on_imu(self, message: Imu) -> None:
        if len(self.samples) >= self.max_samples:
            return
        self.samples.append(float(message.angular_velocity.z))
        self.writer.writerow([len(self.samples), self.samples[-1], 0.0])
        if len(self.samples) >= self.max_samples:
            self.csv_file.flush()
            self.finish()

    def finish(self) -> None:
        rates = np.asarray(self.samples, dtype=float)
        taus = np.logspace(np.log10(0.04), np.log10(len(rates) * 0.01 / 9.0), self.cluster_points)
        taus_s, adev = allan_deviation(rates, 0.01, taus)
        model = fit_noise_model(taus_s, adev)
        payload = {
            "samples": int(len(rates)),
            "arw_deg_sqrt_h": model.arw,
            "bias_instability_deg_h": model.bias_instability,
            "adev_points": int(len(adev)),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(json.dumps(payload, ensure_ascii=False))

    def destroy_node(self) -> bool:
        if not self.csv_file.closed:
            self.csv_file.close()
        return super().destroy_node()


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_imu_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(ImuSimulator())


def main_allan_identifier(args=None) -> None:
    rclpy.init(args=args)
    _spin(AllanIdentifier())
