"""ROS 2 yaw command guard and deterministic demo simulator."""

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

from .core import YawGuard, YawGuardConfig, shortest_angle, wrap_angle
from .scenario import target_yaw_at


CSV_COLUMNS = (
    "time_s",
    "mode",
    "current_yaw_deg",
    "target_yaw_deg",
    "error_deg",
    "rate_deg_s",
    "limited",
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


class YawCommandGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_yaw_command_guard")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_yaw_rate_deg_s", 90.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("current_timeout_s", 1.0)
        self.declare_parameter("output_csv", "/tmp/uav_yaw_command_guard_runtime.csv")
        config = YawGuardConfig(
            control_rate_hz=float(self.get_parameter("control_rate_hz").value),
            max_yaw_rate_deg_s=float(
                self.get_parameter("max_yaw_rate_deg_s").value
            ),
            target_timeout_s=float(self.get_parameter("target_timeout_s").value),
            current_timeout_s=float(
                self.get_parameter("current_timeout_s").value
            ),
        )
        self.guard = YawGuard(config)
        self.started_at = self._monotonic_s()
        self.last_mode: str | None = None
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(CSV_COLUMNS)
        self.csv_file.flush()

        self.rate_pub = self.create_publisher(Twist, "/uav/yaw/rate_cmd", 10)
        self.status_pub = self.create_publisher(
            String, "/uav/yaw/guard_status", 10
        )
        self.create_subscription(
            Float64, "/uav/yaw/current", self.on_current_yaw, 10
        )
        self.create_subscription(
            Float64, "/uav/yaw/target", self.on_target_yaw, 10
        )
        self.create_timer(1.0 / config.control_rate_hz, self.control_step)
        self.get_logger().info(
            "yaw guard started: max_rate=%.1f deg/s rate=%.1f Hz"
            % (config.max_yaw_rate_deg_s, config.control_rate_hz)
        )

    def _monotonic_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_current_yaw(self, msg: Float64) -> None:
        try:
            self.guard.update_current(msg.data, self._monotonic_s())
        except ValueError as exc:
            self.get_logger().warning(f"ignored invalid current yaw: {exc}")

    def on_target_yaw(self, msg: Float64) -> None:
        try:
            self.guard.update_target(msg.data, self._monotonic_s())
        except ValueError as exc:
            self.get_logger().warning(f"ignored invalid target yaw: {exc}")

    def control_step(self) -> None:
        now_s = self._monotonic_s()
        result = self.guard.step(now_s)
        if result is None:
            return
        command = Twist()
        command.angular.z = result.yaw_rate_rad_s
        self.rate_pub.publish(command)
        payload = result.to_dict()
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.writer.writerow(
            (
                f"{now_s - self.started_at:.3f}",
                result.mode,
                f"{math.degrees(result.current_yaw_rad):.3f}",
                f"{math.degrees(result.target_yaw_rad):.3f}",
                f"{math.degrees(result.error_rad):.3f}",
                f"{math.degrees(result.yaw_rate_rad_s):.3f}",
                int(result.limited),
            )
        )
        self.csv_file.flush()
        if result.mode != self.last_mode:
            self.get_logger().info(
                "mode=%s error=%.1f deg rate=%.1f deg/s"
                % (
                    result.mode,
                    math.degrees(result.error_rad),
                    math.degrees(result.yaw_rate_rad_s),
                )
            )
            self.last_mode = result.mode

    def destroy_node(self) -> bool:
        if not self.csv_file.closed:
            self.csv_file.close()
        return super().destroy_node()


class YawDemoSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_yaw_demo_sim")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("plant_yaw_rate_deg_s", 120.0)
        self.rate_hz = _positive_finite(
            self.get_parameter("rate_hz").value, "rate_hz"
        )
        self.duration_s = _non_negative_finite(
            self.get_parameter("duration_s").value, "duration_s"
        )
        self.plant_rate_rad_s = math.radians(
            _positive_finite(
                self.get_parameter("plant_yaw_rate_deg_s").value,
                "plant_yaw_rate_deg_s",
            )
        )
        self.started_at = self.get_clock().now().nanoseconds * 1e-9
        self.current_yaw = 0.0
        self.commanded_rate = 0.0
        self.target_pub = self.create_publisher(
            Float64, "/uav/yaw/target", 10
        )
        self.current_pub = self.create_publisher(
            Float64, "/uav/yaw/current", 10
        )
        self.create_subscription(
            Twist, "/uav/yaw/rate_cmd", self.on_rate_command, 10
        )
        self.create_timer(1.0 / self.rate_hz, self.simulate_step)
        self.get_logger().info("yaw demo simulator started")

    def on_rate_command(self, msg: Twist) -> None:
        self.commanded_rate = msg.angular.z

    def simulate_step(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_at
        target = target_yaw_at(elapsed)
        bounded_command = max(
            -self.plant_rate_rad_s,
            min(self.plant_rate_rad_s, self.commanded_rate),
        )
        self.current_yaw = wrap_angle(
            self.current_yaw + bounded_command / self.rate_hz
        )
        self.target_pub.publish(Float64(data=target))
        self.current_pub.publish(Float64(data=self.current_yaw))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            final_error = shortest_angle(target_yaw_at(elapsed), self.current_yaw)
            self.get_logger().info(
                "yaw scenario completed after %.2f s, final_error=%.1f deg"
                % (elapsed, math.degrees(final_error))
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


def main_guard(args=None) -> None:
    rclpy.init(args=args)
    _spin(YawCommandGuardNode())


def main_simulator(args=None) -> None:
    rclpy.init(args=args)
    _spin(YawDemoSimulator())
