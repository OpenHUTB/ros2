"""ROS 2 link simulator and outage guard nodes."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import LinkGuardConfig, LinkQualityGuard, LinkSample
from .scenario import scenario_at


class LinkQualitySimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_link_quality_sim")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.started_s = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_link/quality", 10)
        self.create_timer(1.0 / self.rate_hz, self.publish_sample)
        self.get_logger().info("link-quality scenario simulator started")

    def publish_sample(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_s
        sample = scenario_at(elapsed)
        sample = LinkSample(
            timestamp_s=elapsed,
            rssi_dbm=sample.rssi_dbm,
            latency_ms=sample.latency_ms,
            packet_loss=sample.packet_loss,
            age_ms=sample.age_ms,
        )
        self.pub.publish(String(data=json.dumps(asdict(sample), ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(f"link scenario completed after {elapsed:.2f} s")
            rclpy.shutdown()


class LinkOutageGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_link_outage_guard")
        for name, value in (
            ("degraded_risk", 0.35),
            ("hold_risk", 0.62),
            ("return_risk", 0.78),
            ("switch_frames", 3),
            ("stale_age_ms", 1600.0),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("output_csv", "/tmp/uav_link_outage_runtime.csv")
        config = LinkGuardConfig(
            degraded_risk=float(self.get_parameter("degraded_risk").value),
            hold_risk=float(self.get_parameter("hold_risk").value),
            return_risk=float(self.get_parameter("return_risk").value),
            switch_frames=int(self.get_parameter("switch_frames").value),
            stale_age_ms=float(self.get_parameter("stale_age_ms").value),
        )
        self.guard = LinkQualityGuard(config)
        self.assessment_pub = self.create_publisher(String, "/uav_link/assessment", 10)
        self.advice_pub = self.create_publisher(String, "/uav_link/advice", 10)
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "time_s",
                "state",
                "risk",
                "rssi_dbm",
                "latency_ms",
                "packet_loss",
                "age_ms",
                "trend_db_per_s",
                "recommendation",
            ]
        )
        self.csv_file.flush()
        self.last_state = None
        self.create_subscription(String, "/uav_link/quality", self.on_quality, 10)
        self.get_logger().info("link degradation guard started")

    def on_quality(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            sample = LinkSample(**payload)
            assessment = self.guard.evaluate(sample)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid link sample: {exc}")
            return
        self.assessment_pub.publish(
            String(data=json.dumps(assessment.to_dict(), ensure_ascii=False))
        )
        advice = {
            "state": assessment.state,
            "recommendation": assessment.recommendation,
            "risk": round(assessment.risk, 4),
            "time_to_horizon_s": assessment.time_to_horizon_s,
        }
        self.advice_pub.publish(String(data=json.dumps(advice, ensure_ascii=False)))
        self.writer.writerow(
            [
                f"{sample.timestamp_s:.3f}",
                assessment.state,
                f"{assessment.risk:.4f}",
                f"{sample.rssi_dbm:.2f}",
                f"{sample.latency_ms:.2f}",
                f"{sample.packet_loss:.3f}",
                f"{sample.age_ms:.1f}",
                f"{assessment.trend_db_per_s:.3f}",
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
    _spin(LinkQualitySimulator())


def main_guard(args=None) -> None:
    rclpy.init(args=args)
    _spin(LinkOutageGuardNode())
