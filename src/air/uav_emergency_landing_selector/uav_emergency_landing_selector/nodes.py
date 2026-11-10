"""ROS 2 simulator and selector nodes for emergency landing assessment."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import LandingConstraints, LandingSite, LandingSiteSelector, UAVState
from .scenario import scenario_at


def _payload(uav: UAVState, sites: list[LandingSite]) -> dict:
    return {"uav": asdict(uav), "sites": [asdict(site) for site in sites]}


def _parse_payload(payload: dict) -> tuple[UAVState, list[LandingSite]]:
    uav = UAVState(**payload["uav"])
    sites = [LandingSite(**item) for item in payload["sites"]]
    return uav, sites


class LandingSiteSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_landing_site_sim")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(0.2, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.started_s = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_landing/sites", 10)
        self.create_timer(1.0 / self.rate_hz, self.publish_scene)
        self.get_logger().info("emergency landing scenario simulator started")

    def publish_scene(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_s - self.started_s
        uav, sites = scenario_at(elapsed)
        self.pub.publish(String(data=json.dumps(_payload(uav, sites), ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.get_logger().info(f"scenario completed after {elapsed:.2f} s")
            rclpy.shutdown()


class LandingSiteSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_landing_site_selector")
        for name, value in (
            ("max_slope_deg", 15.0),
            ("max_roughness_m", 0.45),
            ("max_obstacle_risk", 0.70),
            ("max_people_risk", 0.40),
            ("max_distance_m", 80.0),
            ("reserve_margin_m", 12.0),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("output_csv", "/tmp/uav_landing_selector_runtime.csv")
        self.selector = LandingSiteSelector(
            LandingConstraints(
                **{name: float(self.get_parameter(name).value) for name, _ in (
                    ("max_slope_deg", 0.0),
                    ("max_roughness_m", 0.0),
                    ("max_obstacle_risk", 0.0),
                    ("max_people_risk", 0.0),
                    ("max_distance_m", 0.0),
                    ("reserve_margin_m", 0.0),
                )}
            )
        )
        self.selection_pub = self.create_publisher(String, "/uav_landing/selection", 10)
        self.status_pub = self.create_publisher(String, "/uav_landing/status", 10)
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            ["time_s", "state", "selected_site", "score", "eligible_count", "x", "y"]
        )
        self.csv_file.flush()
        self.last_state = None
        self.create_subscription(String, "/uav_landing/sites", self.on_sites, 10)
        self.get_logger().info("constraint-aware landing site selector started")

    def on_sites(self, msg: String) -> None:
        try:
            uav, sites = _parse_payload(json.loads(msg.data))
            decision = self.selector.select(uav, sites)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid landing scene: {exc}")
            return
        self.selection_pub.publish(String(data=json.dumps(decision.to_dict(), ensure_ascii=False)))
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self.writer.writerow(
            [
                f"{now_s:.3f}",
                decision.state,
                decision.selected_site_id or "",
                "" if decision.score is None else f"{decision.score:.3f}",
                decision.eligible_count,
                "" if decision.selected_x is None else f"{decision.selected_x:.3f}",
                "" if decision.selected_y is None else f"{decision.selected_y:.3f}",
            ]
        )
        self.csv_file.flush()
        status = {
            "state": decision.state,
            "selected_site_id": decision.selected_site_id,
            "score": decision.score,
            "eligible_count": decision.eligible_count,
            "rejected": {
                item.site_id: list(item.rejection_reasons)
                for item in decision.evaluations
                if not item.eligible
            },
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if decision.state != self.last_state:
            self.get_logger().info(
                f"state={decision.state} selected={decision.selected_site_id} "
                f"eligible={decision.eligible_count}"
            )
            self.last_state = decision.state

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
    _spin(LandingSiteSimulator())


def main_selector(args=None) -> None:
    rclpy.init(args=args)
    _spin(LandingSiteSelectorNode())
