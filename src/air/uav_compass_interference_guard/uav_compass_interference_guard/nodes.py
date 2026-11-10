"""ROS 2 magnetometer-interference simulator and guard nodes."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import CompassGuardConfig, CompassInterferenceGuard, CompassSample
from .scenario import scenario_at


class CompassScenarioSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_compass_sim")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.started_s = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_compass/sample", 10)
        self.create_timer(1.0 / self.rate_hz, self.publish_sample)
        self.get_logger().info("compass interference scenario simulator started")

    def publish_sample(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_s
        sample = scenario_at(elapsed)
        self.pub.publish(String(data=json.dumps(asdict(sample), ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(f"compass scenario completed after {elapsed:.2f} s")
            rclpy.shutdown()


class CompassInterferenceGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_compass_interference_guard")
        for name, value in (
            ("suspect_risk", 0.35),
            ("biased_risk", 0.60),
            ("lost_risk", 0.82),
            ("switch_frames", 3),
            ("min_gps_speed_mps", 3.0),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("output_csv", "/tmp/uav_compass_runtime.csv")
        config = CompassGuardConfig(
            suspect_risk=float(self.get_parameter("suspect_risk").value),
            biased_risk=float(self.get_parameter("biased_risk").value),
            lost_risk=float(self.get_parameter("lost_risk").value),
            switch_frames=int(self.get_parameter("switch_frames").value),
            min_gps_speed_mps=float(self.get_parameter("min_gps_speed_mps").value),
        )
        self.guard = CompassInterferenceGuard(config)
        self.assessment_pub = self.create_publisher(
            String, "/uav_compass/assessment", 10
        )
        self.advice_pub = self.create_publisher(String, "/uav_compass/advice", 10)
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "time_s",
                "state",
                "risk",
                "field_ut",
                "field_dev_ut",
                "gps_diff_deg",
                "vio_diff_deg",
                "current_a",
                "altitude_m",
                "recommendation",
            ]
        )
        self.csv_file.flush()
        self.last_state = None
        self.create_subscription(String, "/uav_compass/sample", self.on_sample, 10)
        self.get_logger().info("compass interference guard started")

    def on_sample(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            sample = CompassSample(**payload)
            assessment = self.guard.evaluate(sample)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid compass sample: {exc}")
            return
        self.assessment_pub.publish(
            String(data=json.dumps(assessment.to_dict(), ensure_ascii=False))
        )
        advice = {
            "state": assessment.state,
            "recommendation": assessment.recommendation,
            "risk": round(assessment.risk, 4),
            "gps_heading_diff_deg": round(assessment.gps_heading_diff_deg, 2),
            "vio_heading_diff_deg": round(assessment.vio_heading_diff_deg, 2),
        }
        self.advice_pub.publish(String(data=json.dumps(advice, ensure_ascii=False)))
        self.writer.writerow(
            [
                f"{sample.timestamp_s:.3f}",
                assessment.state,
                f"{assessment.risk:.4f}",
                f"{sample.mag_field_ut:.2f}",
                f"{assessment.field_deviation_ut:.2f}",
                f"{assessment.gps_heading_diff_deg:.2f}",
                f"{assessment.vio_heading_diff_deg:.2f}",
                f"{sample.motor_current_a:.2f}",
                f"{sample.altitude_m:.2f}",
                assessment.recommendation,
            ]
        )
        self.csv_file.flush()
        if assessment.state != self.last_state:
            self.get_logger().info(
                f"state={assessment.state} risk={assessment.risk:.3f} "
                f"rec={assessment.recommendation}"
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
    _spin(CompassScenarioSimulator())


def main_guard(args=None) -> None:
    rclpy.init(args=args)
    _spin(CompassInterferenceGuardNode())
