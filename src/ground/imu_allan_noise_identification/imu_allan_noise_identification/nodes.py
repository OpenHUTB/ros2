"""ROS 2 nodes: an IMU replay source and an online Allan-variance identifier.

Units follow ``sensor_msgs/Imu``: angular velocity in rad/s, linear acceleration
in m/s^2, orientation as a unit quaternion.  The simulated motion is a slow roll
sweep, so the gyro measures the roll rate on x and the accelerometer sees gravity
rotated about x -- the two channels stay on the same axis.
"""

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

GRAVITY = 9.80665


def _quaternion_from_roll(roll: float):
    """Return ``(x, y, z, w)`` of a pure roll rotation."""
    return float(np.sin(roll / 2.0)), 0.0, 0.0, float(np.cos(roll / 2.0))


class ImuSimulator(Node):
    """Replay the deterministic roll sweep as a ``sensor_msgs/Imu`` stream."""

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
            f"imu_sim ready: {len(self.sweep['time_s'])} samples at {self.rate_hz:.0f} Hz, "
            f"starting epoch {self.epoch}"
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
        roll = float(np.deg2rad(self.sweep["tilt_deg"][index]))
        message = Imu()
        message.header.frame_id = "imu_link"
        message.header.stamp = self.get_clock().now().to_msg()
        # rad/s about x, matching the roll motion of the orientation field.
        message.angular_velocity.x = float(np.deg2rad(self.sweep["gyro_deg_s"][index]))
        quaternion = _quaternion_from_roll(roll)
        message.orientation.x = quaternion[0]
        message.orientation.y = quaternion[1]
        message.orientation.z = quaternion[2]
        message.orientation.w = quaternion[3]
        # Specific force of a stationary sensor tilted about x, in m/s^2.
        message.linear_acceleration.x = 0.0
        message.linear_acceleration.y = float(GRAVITY * np.sin(roll))
        message.linear_acceleration.z = float(GRAVITY * np.cos(roll))
        self.publisher.publish(message)
        self.epoch += 1


class AllanIdentifier(Node):
    """Collect a static gyro log, then identify ARW and bias instability."""

    def __init__(self) -> None:
        super().__init__("allan_identifier")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("max_samples", 12000)
        self.declare_parameter("cluster_points", 20)
        self.declare_parameter("output_csv", "/tmp/allan_identifier.csv")
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.max_samples = int(self.get_parameter("max_samples").value)
        self.cluster_points = int(self.get_parameter("cluster_points").value)
        self.samples: list[float] = []
        self.finished = False
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file, lineterminator="\n")
        self.writer.writerow(["index", "gyro_x_rad_s"])
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)
        self.status_pub = self.create_publisher(String, "/imu/allan", 10)
        self.get_logger().info(
            f"allan_identifier ready: {self.rate_hz:.0f} Hz, waiting for "
            f"{self.max_samples} samples ({self.max_samples / self.rate_hz:.0f} s)"
        )

    def on_imu(self, message: Imu) -> None:
        if self.finished or len(self.samples) >= self.max_samples:
            return
        self.samples.append(float(np.rad2deg(message.angular_velocity.x)))
        self.writer.writerow([len(self.samples), f"{self.samples[-1]:.9f}"])
        if len(self.samples) >= self.max_samples:
            self.csv_file.flush()
            self.finish()

    def finish(self) -> None:
        """Identify from whatever was collected; warn (do not crash) if sparse."""
        if self.finished:
            return
        self.finished = True
        dt = 1.0 / self.rate_hz
        rates = np.asarray(self.samples, dtype=float)
        if rates.size < 100:
            self.get_logger().warning(
                f"only {rates.size} samples collected, not enough for a stable fit"
            )
            return
        taus = np.logspace(np.log10(4.0 * dt), np.log10(rates.size * dt / 9.0),
                           self.cluster_points)
        taus_s, adev = allan_deviation(rates, dt, taus)
        model = fit_noise_model(taus_s, adev)
        max_tau_s = float(taus_s[-1])
        bias_reliable = max_tau_s >= 100.0 and model.bias_instability > 0.0
        payload = {
            "samples": int(rates.size),
            "rate_hz": self.rate_hz,
            "arw_deg_sqrt_h": model.arw,
            "bias_instability_deg_h": model.bias_instability,
            "max_tau_s": max_tau_s,
            "bias_reliable": bias_reliable,
            "adev_points": int(len(adev)),
        }
        if not bias_reliable:
            reason = (
                f"longest cluster time is only {max_tau_s:.1f} s (bias instability needs >= 1 h)"
                if max_tau_s < 100.0
                else "the fitted bias-instability term was clipped to zero"
            )
            self.get_logger().warning(
                "bias instability NOT identifiable from this recording: "
                f"{reason}; reported value {model.bias_instability:.3f} deg/h is not usable"
            )
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(json.dumps(payload, ensure_ascii=False))

    def destroy_node(self) -> bool:
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        if not self.finished and self.samples:
            self.get_logger().info("identifying from the samples collected before shutdown")
            self.finish()
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
