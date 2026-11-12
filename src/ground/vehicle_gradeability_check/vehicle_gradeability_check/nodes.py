"""ROS 2 gradeability checker and deterministic demo simulator."""

from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
import signal

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .core import GradeabilityChecker, GradeabilityConfig
from .scenario import slope_at, speed_at


CSV_COLUMNS = (
    "time_s",
    "mode",
    "slope_deg",
    "speed_mps",
    "required_n",
    "drive_limit_n",
    "traction_limit_n",
    "available_n",
    "margin_n",
    "slip_risk",
    "power_deficit",
)


def _positive_finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return value


def _non_negative_finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


class GradeabilityCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_gradeability_check")
        self.declare_parameter("mass_kg", 1500.0)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("crr", 0.012)
        self.declare_parameter("cda", 0.72)
        self.declare_parameter("air_density", 1.225)
        self.declare_parameter("max_drive_force_n", 4500.0)
        self.declare_parameter("friction_coef", 0.6)
        self.declare_parameter("slope_timeout_s", 1.0)
        self.declare_parameter("speed_timeout_s", 1.0)
        self.declare_parameter(
            "output_csv", "/tmp/vehicle_gradeability_check_runtime.csv"
        )
        config = GradeabilityConfig(
            mass_kg=float(self.get_parameter("mass_kg").value),
            gravity=float(self.get_parameter("gravity").value),
            crr=float(self.get_parameter("crr").value),
            cda=float(self.get_parameter("cda").value),
            air_density=float(self.get_parameter("air_density").value),
            max_drive_force_n=float(
                self.get_parameter("max_drive_force_n").value
            ),
            friction_coef=float(self.get_parameter("friction_coef").value),
            slope_timeout_s=float(self.get_parameter("slope_timeout_s").value),
            speed_timeout_s=float(self.get_parameter("speed_timeout_s").value),
        )
        self.checker = GradeabilityChecker(config)
        self.started_at = self._monotonic_s()
        self.last_mode: str | None = None
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(CSV_COLUMNS)
        self.csv_file.flush()

        self.cmd_pub = self.create_publisher(
            Twist, "/vehicle/grade/traction_cmd", 10
        )
        self.status_pub = self.create_publisher(
            String, "/vehicle/grade/grade_status", 10
        )
        self.create_subscription(
            Float64, "/vehicle/grade/slope", self.on_slope, 10
        )
        self.create_subscription(
            Float64, "/vehicle/grade/speed", self.on_speed, 10
        )
        self.create_timer(
            1.0 / config.control_rate_hz, self.control_step
        )
        self.get_logger().info(
            "gradeability checker started: mass=%.0f kg max_drive=%.0f N mu=%.2f"
            % (config.mass_kg, config.max_drive_force_n, config.friction_coef)
        )

    def _monotonic_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_slope(self, msg: Float64) -> None:
        try:
            self.checker.update_slope(msg.data, self._monotonic_s())
        except ValueError as exc:
            self.get_logger().warning(f"ignored invalid slope: {exc}")

    def on_speed(self, msg: Float64) -> None:
        try:
            self.checker.update_speed(msg.data, self._monotonic_s())
        except ValueError as exc:
            self.get_logger().warning(f"ignored invalid speed: {exc}")

    def control_step(self) -> None:
        now_s = self._monotonic_s()
        result = self.checker.step(now_s)
        if result is None:
            return
        command = Twist()
        command.linear.x = max(0.0, min(result.available_n, result.required_n))
        self.cmd_pub.publish(command)
        self.status_pub.publish(String(data=json.dumps(result.to_dict())))
        self.writer.writerow(
            (
                f"{now_s - self.started_at:.3f}",
                result.mode,
                f"{result.slope_deg:.3f}",
                f"{result.speed_mps:.3f}",
                f"{result.required_n:.3f}",
                f"{result.drive_limit_n:.3f}",
                f"{result.traction_limit_n:.3f}",
                f"{result.available_n:.3f}",
                f"{result.margin_n:.3f}",
                int(result.slip_risk),
                int(result.power_deficit),
            )
        )
        self.csv_file.flush()
        if result.mode != self.last_mode:
            self.get_logger().info(
                "mode=%s slope=%.1f deg required=%.0f N margin=%.0f N"
                % (result.mode, result.slope_deg, result.required_n, result.margin_n)
            )
            self.last_mode = result.mode

    def destroy_node(self) -> bool:
        if not self.csv_file.closed:
            self.csv_file.close()
        return super().destroy_node()


class GradeabilityDemoSimulator(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_gradeability_demo_sim")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("duration_s", 10.0)
        self.rate_hz = _positive_finite(
            self.get_parameter("rate_hz").value, "rate_hz"
        )
        self.duration_s = _non_negative_finite(
            self.get_parameter("duration_s").value, "duration_s"
        )
        self.started_at = self.get_clock().now().nanoseconds * 1e-9
        self.last_command = 0.0
        self.slope_pub = self.create_publisher(
            Float64, "/vehicle/grade/slope", 10
        )
        self.speed_pub = self.create_publisher(
            Float64, "/vehicle/grade/speed", 10
        )
        self.create_subscription(
            Twist, "/vehicle/grade/traction_cmd", self.on_command, 10
        )
        self.create_timer(1.0 / self.rate_hz, self.simulate_step)
        self.get_logger().info("gradeability demo simulator started")

    def on_command(self, msg: Twist) -> None:
        self.last_command = msg.linear.x

    def simulate_step(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_at
        self.slope_pub.publish(Float64(data=slope_at(elapsed)))
        self.speed_pub.publish(Float64(data=speed_at(elapsed)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(
                "gradeability scenario completed after %.2f s, last_command=%.0f N"
                % (elapsed, self.last_command)
            )
            os.kill(os.getpid(), signal.SIGINT)


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_checker(args=None) -> None:
    rclpy.init(args=args)
    _spin(GradeabilityCheckNode())


def main_simulator(args=None) -> None:
    rclpy.init(args=args)
    _spin(GradeabilityDemoSimulator())
