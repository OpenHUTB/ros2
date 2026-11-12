"""ROS 2 nodes: quantised encoder stream and velocity estimators."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from .core import VelocityFilter, alpha_beta, moving_average, raw_difference
from .scene import ProfileConfig, measure, trapezoid


class EncoderSimulator(Node):
    """Publish the quantised joint position of a trapezoidal profile."""

    def __init__(self) -> None:
        super().__init__("encoder_sim")
        self.declare_parameter("counts_per_revolution", 4096)
        self.resolution = int(self.get_parameter("counts_per_revolution").value)
        profile = trapezoid(ProfileConfig())
        self.data = measure(profile, self.resolution)
        self.publisher = self.create_publisher(JointState, "/joint/encoder", 50)
        self.declare_parameter("duration_s", 0.0)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.index = 0
        self.timer = self.create_timer(self.data["dt"], self.step)
        self.get_logger().info(
            f"encoder_sim ready: {self.resolution} CPR, step {self.data['step_rad']:.5f} rad"
        )

    def step(self) -> None:
        if self.duration_s > 0.0 and self.index >= self.duration_s / self.data["dt"]:
            self.timer.cancel()
            self.get_logger().info(f"replayed {self.index} samples, stopping the timer")
            return
        index = self.index % len(self.data["position_rad"])
        message = JointState()
        message.header.frame_id = "joint"
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["joint"]
        message.position = [float(self.data["position_rad"][index])]
        self.publisher.publish(message)
        self.index += 1


class VelocityEstimator(Node):
    """Raw difference, moving average and Kalman velocity side by side."""

    def __init__(self) -> None:
        super().__init__("velocity_estimator")
        self.declare_parameter("dt", 0.001)
        self.declare_parameter("window", 5)
        self.declare_parameter("counts_per_revolution", 4096)
        self.dt = float(self.get_parameter("dt").value)
        self.window_size = max(1, int(self.get_parameter("window").value))
        self.counts_per_revolution = int(
            self.get_parameter("counts_per_revolution").value
        )
        self.positions: list[float] = []
        self.window: list[float] = []
        self.alpha_beta_state = {"position": None, "velocity": 0.0}
        # alpha-beta gains follow the offline script (alpha=0.35, beta=0.08)
        self.alpha, self.beta = 0.35, 0.08
        self.step_rad = 2.0 * np.pi / self.counts_per_revolution
        self.filter = VelocityFilter.create(
            process_sigma=25.0, measurement_sigma=self.step_rad / 3.0
        )
        self.raw_pub = self.create_publisher(Float64, "/joint/velocity_raw", 50)
        self.filtered_pub = self.create_publisher(Float64, "/joint/velocity", 50)
        self.status_pub = self.create_publisher(String, "/joint/velocity_status", 10)
        self.create_subscription(JointState, "/joint/encoder", self.on_encoder, 50)
        self.get_logger().info(f"velocity_estimator ready: dt {self.dt}, window {self.window}")

    def on_encoder(self, message: JointState) -> None:
        if not message.position:
            return
        self.positions.append(float(message.position[0]))
        raw = Float64()
        raw.data = 0.0 if len(self.positions) < 2 else (
            self.positions[-1] - self.positions[-2]
        ) / self.dt
        self.raw_pub.publish(raw)
        smoothed = float(self.filter.step(self.positions[-1], self.dt))
        estimate = Float64()
        estimate.data = smoothed
        self.filtered_pub.publish(estimate)
        state = self.alpha_beta_state
        if state["position"] is None:
            state["position"] = self.positions[-1]
        predicted = state["position"] + state["velocity"] * self.dt
        residual = self.positions[-1] - predicted
        state["position"] = predicted + self.alpha * residual
        state["velocity"] += self.beta * residual / self.dt
        self.alpha_beta_velocity = state["velocity"]
        # Fixed-length ring buffer keeps the status window O(1) per sample.
        self.window.append(raw.data)
        if len(self.window) > self.window_size:
            self.window.pop(0)
        if len(self.positions) % 500 == 0:
            window = moving_average(np.asarray(self.window), self.window_size)
            payload = {
                "raw": raw.data,
                "kalman": smoothed,
                "moving_average": float(window[-1]) if window.size else 0.0,
                "alpha_beta": self.alpha_beta_velocity,
                "samples": len(self.positions),
            }
            self.status_pub.publish(String(data=json.dumps(payload)))


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_encoder_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(EncoderSimulator())


def main_velocity_estimator(args=None) -> None:
    rclpy.init(args=args)
    _spin(VelocityEstimator())
