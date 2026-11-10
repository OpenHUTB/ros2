"""ROS 2 moving-geofence simulator and predictive guard."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import DynamicZone, GeofenceConfig, GeofenceGuard, UAVMotion
from .scenario import scenario_at


class DynamicGeofenceSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_dynamic_geofence_sim")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.started_s = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_geofence/scene", 10)
        self.create_timer(1.0 / self.rate_hz, self.publish_scene)
        self.get_logger().info("moving geofence scenario simulator started")

    def publish_scene(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_s
        uav, zones = scenario_at(elapsed)
        payload = {
            "uav": asdict(uav),
            "zones": [asdict(zone) for zone in zones],
        }
        self.pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(f"geofence scenario completed after {elapsed:.2f} s")
            rclpy.shutdown()


class DynamicGeofenceGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_dynamic_geofence_guard")
        for name, value in (
            ("horizon_s", 8.0),
            ("warning_margin_m", 5.0),
            ("critical_margin_m", 1.5),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("output_csv", "/tmp/uav_dynamic_geofence_runtime.csv")
        self.guard = GeofenceGuard(
            GeofenceConfig(
                horizon_s=float(self.get_parameter("horizon_s").value),
                warning_margin_m=float(self.get_parameter("warning_margin_m").value),
                critical_margin_m=float(self.get_parameter("critical_margin_m").value),
            )
        )
        self.assessment_pub = self.create_publisher(
            String, "/uav_geofence/assessment", 10
        )
        self.advice_pub = self.create_publisher(String, "/uav_geofence/advice", 10)
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "time_s",
                "state",
                "zone",
                "clearance_m",
                "time_to_closest_s",
                "avoid_x",
                "avoid_y",
                "recommendation",
            ]
        )
        self.csv_file.flush()
        self.last_state = None
        self.create_subscription(String, "/uav_geofence/scene", self.on_scene, 10)
        self.get_logger().info("predictive dynamic geofence guard started")

    def on_scene(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            uav = UAVMotion(**payload["uav"])
            zones = [DynamicZone(**item) for item in payload["zones"]]
            assessment = self.guard.assess(uav, zones)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid geofence scene: {exc}")
            return
        self.assessment_pub.publish(
            String(data=json.dumps(assessment.to_dict(), ensure_ascii=False))
        )
        advice = {
            "state": assessment.state,
            "zone": assessment.limiting_zone_id,
            "recommendation": assessment.recommendation,
            "avoid_vector": [assessment.avoid_x, assessment.avoid_y],
        }
        self.advice_pub.publish(String(data=json.dumps(advice, ensure_ascii=False)))
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self.writer.writerow(
            [
                f"{now_s:.3f}",
                assessment.state,
                assessment.limiting_zone_id or "",
                ""
                if assessment.predicted_clearance_m is None
                else f"{assessment.predicted_clearance_m:.3f}",
                ""
                if assessment.time_to_closest_s is None
                else f"{assessment.time_to_closest_s:.3f}",
                f"{assessment.avoid_x:.4f}",
                f"{assessment.avoid_y:.4f}",
                assessment.recommendation,
            ]
        )
        self.csv_file.flush()
        if assessment.state != self.last_state:
            clearance = (
                "n/a"
                if assessment.predicted_clearance_m is None
                else f"{assessment.predicted_clearance_m:.2f} m"
            )
            self.get_logger().info(
                f"state={assessment.state} zone={assessment.limiting_zone_id} "
                f"clearance={clearance} recommendation={assessment.recommendation}"
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
    _spin(DynamicGeofenceSimulator())


def main_guard(args=None) -> None:
    rclpy.init(args=args)
    _spin(DynamicGeofenceGuardNode())
