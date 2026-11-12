"""ROS 2 payload-balance simulator and monitor."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import PayloadBalanceConfig, PayloadBalanceGuard, PayloadSample
from .scenario import scenario_at


class PayloadBalanceSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_payload_balance_sim")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.started_s = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_payload/balance_sample", 10)
        self.create_timer(1.0 / self.rate_hz, self.publish_sample)
        self.get_logger().info("payload balance scenario simulator started")

    def publish_sample(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_s
        self.pub.publish(
            String(data=json.dumps(asdict(scenario_at(elapsed)), ensure_ascii=False))
        )
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(f"payload scenario completed after {elapsed:.2f} s")
            rclpy.shutdown()


class PayloadBalanceGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_payload_balance_guard")
        for name, value in (
            ("adjust_risk", 0.35),
            ("warning_risk", 0.60),
            ("unstable_risk", 0.80),
            ("switch_frames", 3),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("output_csv", "/tmp/uav_payload_balance_runtime.csv")
        self.guard = PayloadBalanceGuard(
            PayloadBalanceConfig(
                adjust_risk=float(self.get_parameter("adjust_risk").value),
                warning_risk=float(self.get_parameter("warning_risk").value),
                unstable_risk=float(self.get_parameter("unstable_risk").value),
                switch_frames=int(self.get_parameter("switch_frames").value),
            )
        )
        self.assessment_pub = self.create_publisher(
            String, "/uav_payload/balance_assessment", 10
        )
        self.advice_pub = self.create_publisher(
            String, "/uav_payload/balance_advice", 10
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
                "thrust_imbalance_ratio",
                "estimated_cg_offset_m",
                "roll_correction_deg",
                "sway_rate_mps",
                "payload_ratio",
                "recommendation",
            ]
        )
        self.csv_file.flush()
        self.last_state = None
        self.create_subscription(
            String, "/uav_payload/balance_sample", self.on_sample, 10
        )
        self.get_logger().info("payload balance guard started")

    def on_sample(self, msg: String) -> None:
        try:
            sample = PayloadSample(**json.loads(msg.data))
            assessment = self.guard.evaluate(sample)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid payload sample: {exc}")
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
                f"{assessment.thrust_imbalance_ratio:.4f}",
                f"{assessment.estimated_cg_offset_m:.4f}",
                f"{assessment.roll_correction_deg:.2f}",
                f"{assessment.sway_rate_mps:.3f}",
                f"{assessment.payload_ratio:.4f}",
                assessment.recommendation,
            ]
        )
        self.csv_file.flush()
        if assessment.state != self.last_state:
            self.get_logger().info(
                f"state={assessment.state} risk={assessment.risk:.3f} "
                f"cg_offset={assessment.estimated_cg_offset_m:.3f} m "
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
    _spin(PayloadBalanceSimulator())


def main_guard(args=None) -> None:
    rclpy.init(args=args)
    _spin(PayloadBalanceGuardNode())
