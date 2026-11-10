"""ROS 2 GPS simulator and integrity guard nodes."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import GpsIntegrityConfig, GpsIntegrityGuard, GpsSample
from .scenario import scenario_at


class GpsIntegritySimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_gps_integrity_sim")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.started_s = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_gps/integrity_sample", 10)
        self.create_timer(1.0 / self.rate_hz, self.publish_sample)
        self.get_logger().info("GPS spoofing scenario simulator started")

    def publish_sample(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_s
        sample = scenario_at(elapsed)
        scalar = asdict(sample)
        scalar["x_m"] = scalar.pop("x_m")
        self.pub.publish(String(data=json.dumps(scalar, ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(f"GPS scenario completed after {elapsed:.2f} s")
            rclpy.shutdown()


class GpsIntegrityGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_gps_integrity_guard")
        self.declare_parameter("suspect_risk", 0.35)
        self.declare_parameter("reject_risk", 0.68)
        self.declare_parameter("switch_frames", 2)
        self.declare_parameter("output_csv", "/tmp/uav_gps_integrity_runtime.csv")
        self.guard = GpsIntegrityGuard(
            GpsIntegrityConfig(
                suspect_risk=float(self.get_parameter("suspect_risk").value),
                reject_risk=float(self.get_parameter("reject_risk").value),
                switch_frames=int(self.get_parameter("switch_frames").value),
            )
        )
        self.assessment_pub = self.create_publisher(
            String, "/uav_gps/integrity_assessment", 10
        )
        self.advice_pub = self.create_publisher(String, "/uav_gps/integrity_advice", 10)
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "time_s",
                "state",
                "risk",
                "position_speed_mps",
                "speed_residual_mps",
                "imu_residual_mps",
                "accuracy_m",
                "recommendation",
            ]
        )
        self.csv_file.flush()
        self.last_state = None
        self.create_subscription(
            String, "/uav_gps/integrity_sample", self.on_sample, 10
        )
        self.get_logger().info("GPS integrity guard started")

    def on_sample(self, msg: String) -> None:
        try:
            sample = GpsSample(**json.loads(msg.data))
            assessment = self.guard.evaluate(sample)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid GPS sample: {exc}")
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
                f"{assessment.position_speed_mps:.3f}",
                f"{assessment.speed_residual_mps:.3f}",
                f"{assessment.imu_residual_mps:.3f}",
                f"{assessment.accuracy_m:.3f}",
                assessment.recommendation,
            ]
        )
        self.csv_file.flush()
        if assessment.state != self.last_state:
            self.get_logger().info(
                f"state={assessment.state} risk={assessment.risk:.3f} "
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
    _spin(GpsIntegritySimulator())


def main_guard(args=None) -> None:
    rclpy.init(args=args)
    _spin(GpsIntegrityGuardNode())
