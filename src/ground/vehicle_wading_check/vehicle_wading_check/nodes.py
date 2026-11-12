"""ROS 2 wading checker node and deterministic wading simulator."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .core import WadingChecker, WadingConfig
from .demo import CSV_COLUMNS
from .scenario import depth_at, speed_at


def _positive_finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return value


class WadingCheckerNode(Node):
    """Subscribe to depth/speed, publish a JSON status and optionally a CSV."""

    def __init__(self) -> None:
        super().__init__("wading_checker")
        self.declare_parameter("mass_kg", 1500.0)
        self.declare_parameter("intake_height_m", 0.65)
        self.declare_parameter("exhaust_height_m", 0.35)
        self.declare_parameter("door_sill_height_m", 0.40)
        self.declare_parameter("displacement_volume_factor", 0.35)
        self.declare_parameter("friction_coef_dry", 0.60)
        self.declare_parameter("friction_coef_wet", 0.35)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("csv_path", "")

        rate_hz = _positive_finite(
            self.get_parameter("control_rate_hz").value, "control_rate_hz"
        )
        self._config = WadingConfig(
            mass_kg=float(self.get_parameter("mass_kg").value),
            intake_height_m=float(self.get_parameter("intake_height_m").value),
            exhaust_height_m=float(self.get_parameter("exhaust_height_m").value),
            door_sill_height_m=float(self.get_parameter("door_sill_height_m").value),
            displacement_volume_factor=float(
                self.get_parameter("displacement_volume_factor").value
            ),
            friction_coef_dry=float(self.get_parameter("friction_coef_dry").value),
            friction_coef_wet=float(self.get_parameter("friction_coef_wet").value),
            control_rate_hz=rate_hz,
        )
        self._checker = WadingChecker(self._config)
        self._t0 = self.get_clock().now().nanoseconds / 1e9

        self._status_pub = self.create_publisher(String, "/wading/status", 10)
        self.create_subscription(
            Float64, "/vehicle/wading_depth_m", self._on_depth, 10
        )
        self.create_subscription(Float64, "/vehicle/speed_mps", self._on_speed, 10)

        csv_path = str(self.get_parameter("csv_path").value)
        self._csv_path = Path(csv_path) if csv_path else None
        self._csv_handle = None
        self._csv_writer = None
        if self._csv_path is not None:
            self._csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_handle = self._csv_path.open("w", newline="", encoding="utf-8")
            self._csv_writer = csv.DictWriter(self._csv_handle, fieldnames=CSV_COLUMNS)
            self._csv_writer.writeheader()

        self.create_timer(1.0 / rate_hz, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_depth(self, msg: Float64) -> None:
        if not math.isfinite(msg.data):
            return
        self._checker.update_depth(max(0.0, msg.data), self._now())

    def _on_speed(self, msg: Float64) -> None:
        if not math.isfinite(msg.data):
            return
        self._checker.update_speed(max(0.0, msg.data), self._now())

    def _tick(self) -> None:
        now = self._now()
        result = self._checker.step(now)
        if result is None:
            return
        payload = {
            "mode": result.mode,
            "depth_m": round(result.depth_m, 3),
            "speed_mps": round(result.speed_mps, 3),
            "depth_limit_m": round(result.depth_limit_m, 3),
            "safe_depth_m": round(result.safe_depth_m, 3),
            "buoyancy_ratio": round(result.buoyancy_ratio, 4),
            "v_max_mps": round(result.v_max_mps, 3),
            "traction_margin_n": round(result.traction_margin_n, 1),
        }
        self._status_pub.publish(String(data=json.dumps(payload)))
        if self._csv_writer is not None:
            self._csv_writer.writerow(
                {
                    "time_s": round(now - self._t0, 3),
                    "depth_m": round(result.depth_m, 3),
                    "speed_mps": round(result.speed_mps, 3),
                    "mode": result.mode,
                    "buoyancy_ratio": round(result.buoyancy_ratio, 4),
                    "traction_limit_n": round(result.traction_limit_n, 1),
                    "drag_n": round(result.drag_n, 1),
                    "v_max_mps": round(result.v_max_mps, 3),
                    "traction_margin_n": round(result.traction_margin_n, 1),
                }
            )

    def destroy_node(self) -> bool:
        if self._csv_handle is not None:
            self._csv_handle.close()
            self._csv_handle = None
        return super().destroy_node()


class WadingSimNode(Node):
    """Publish the deterministic depth and speed profile."""

    def __init__(self) -> None:
        super().__init__("wading_sim")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("duration_s", 24.0)
        self._rate_hz = _positive_finite(self.get_parameter("rate_hz").value, "rate_hz")
        self._duration_s = _positive_finite(
            self.get_parameter("duration_s").value, "duration_s"
        )
        self._started = self.get_clock().now()
        self._depth_pub = self.create_publisher(Float64, "/vehicle/wading_depth_m", 10)
        self._speed_pub = self.create_publisher(Float64, "/vehicle/speed_mps", 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._started).nanoseconds / 1e9
        if elapsed > self._duration_s:
            self.get_logger().info(
                f"scenario finished after {elapsed:.2f} s, shutting down"
            )
            raise SystemExit(0)
        self._depth_pub.publish(Float64(data=depth_at(elapsed)))
        self._speed_pub.publish(Float64(data=speed_at(elapsed)))


def main_checker(args=None) -> None:
    rclpy.init(args=args)
    node = WadingCheckerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main_simulator(args=None) -> None:
    rclpy.init(args=args)
    node = WadingSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
