"""ROS 2 battery simulator and cell-health monitor."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import BatteryHealthConfig, BatteryHealthGuard, BatterySample
from .scenario import scenario_at


class BatteryHealthSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_battery_health_sim")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.started_s = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_battery/health_sample", 10)
        self.create_timer(1.0 / self.rate_hz, self.publish_sample)
        self.get_logger().info("battery health scenario simulator started")

    def publish_sample(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_s
        sample = scenario_at(elapsed)
        payload = asdict(sample)
        payload["cell_voltages_v"] = list(payload["cell_voltages_v"])
        self.pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(f"battery scenario completed after {elapsed:.2f} s")
            rclpy.shutdown()


class BatteryHealthGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_battery_health_guard")
        for name, value in (
            ("degraded_imbalance_v", 0.06),
            ("limit_imbalance_v", 0.14),
            ("critical_imbalance_v", 0.24),
            ("switch_frames", 3),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("output_csv", "/tmp/uav_battery_health_runtime.csv")
        self.guard = BatteryHealthGuard(
            BatteryHealthConfig(
                degraded_imbalance_v=float(
                    self.get_parameter("degraded_imbalance_v").value
                ),
                limit_imbalance_v=float(
                    self.get_parameter("limit_imbalance_v").value
                ),
                critical_imbalance_v=float(
                    self.get_parameter("critical_imbalance_v").value
                ),
                switch_frames=int(self.get_parameter("switch_frames").value),
            )
        )
        self.assessment_pub = self.create_publisher(
            String, "/uav_battery/health_assessment", 10
        )
        self.advice_pub = self.create_publisher(
            String, "/uav_battery/health_advice", 10
        )
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "time_s",
                "state",
                "risk",
                "imbalance_v",
                "min_cell_v",
                "mean_cell_v",
                "temperature_c",
                "internal_resistance_mohm",
                "recommendation",
            ]
        )
        self.csv_file.flush()
        self.last_state = None
        self.create_subscription(
            String, "/uav_battery/health_sample", self.on_sample, 10
        )
        self.get_logger().info("battery health guard started")

    def on_sample(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            payload["cell_voltages_v"] = tuple(payload["cell_voltages_v"])
            sample = BatterySample(**payload)
            assessment = self.guard.evaluate(sample)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid battery sample: {exc}")
            return
        self.assessment_pub.publish(
            String(data=json.dumps(assessment.to_dict(), ensure_ascii=False))
        )
        self.advice_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": assessment.state,
                        "recommendation": assessment.recommendation,
                        "risk": round(assessment.risk, 4),
                    },
                    ensure_ascii=False,
                )
            )
        )
        self.writer.writerow(
            [
                f"{sample.timestamp_s:.3f}",
                assessment.state,
                f"{assessment.risk:.4f}",
                f"{assessment.imbalance_v:.4f}",
                f"{assessment.min_cell_v:.4f}",
                f"{assessment.mean_cell_v:.4f}",
                f"{assessment.temperature_c:.2f}",
                f"{assessment.internal_resistance_mohm:.2f}",
                assessment.recommendation,
            ]
        )
        self.csv_file.flush()
        if assessment.state != self.last_state:
            self.get_logger().info(
                f"state={assessment.state} imbalance={assessment.imbalance_v:.3f} V "
                f"recommendation={assessment.recommendation}"
            )
            self.last_state = assessment.state

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


def main_simulator(args=None) -> None:
    rclpy.init(args=args)
    _spin(BatteryHealthSimulator())


def main_guard(args=None) -> None:
    rclpy.init(args=args)
    _spin(BatteryHealthGuardNode())
