"""ROS 2 nodes for baro altitude drift watch."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import AltSample, BaroAltitudeDriftWatch, BaroWatchConfig
from .scenario import scenario_at


class AltSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uav_alt_sim")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_alt/sample", 10)
        self.create_timer(1.0 / self.rate_hz, self.pub)
        self.get_logger().info("altitude scenario simulator started")

    def pub(self) -> None:
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        s = scenario_at(elapsed)
        self.pub.publish(String(data=json.dumps(asdict(s), ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            rclpy.shutdown()


class BaroWatchNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_baro_watch")
        self.declare_parameter("output_csv", "/tmp/uav_baro_runtime.csv")
        self.watch = BaroAltitudeDriftWatch()
        self.pub = self.create_publisher(String, "/uav_alt/assessment", 10)
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.f = self.csv_path.open("w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "state", "risk", "bg", "br", "rate"])
        self.f.flush()
        self.last = None
        self.create_subscription(String, "/uav_alt/sample", self.on_sample, 10)

    def on_sample(self, msg: String) -> None:
        try:
            s = AltSample(**json.loads(msg.data))
            a = self.watch.evaluate(s)
        except Exception as exc:
            self.get_logger().warning(f"bad sample: {exc}")
            return
        self.pub.publish(String(data=json.dumps(a.to_dict(), ensure_ascii=False)))
        self.w.writerow([f"{s.timestamp_s:.2f}", a.state, f"{a.risk:.3f}",
                         f"{a.baro_gps_diff_m:.2f}", f"{a.baro_radar_diff_m:.2f}",
                         f"{a.pressure_rate_hpa_s:.3f}"])
        self.f.flush()
        if a.state != self.last:
            self.get_logger().info(f"state={a.state} risk={a.risk:.3f} -> {a.recommendation}")
            self.last = a.state

    def destroy_node(self) -> bool:
        self.f.close()
        return super().destroy_node()


def _spin(n: Node) -> None:
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_sim():
    rclpy.init()
    _spin(AltSimulator())


def main_watch():
    rclpy.init()
    _spin(BaroWatchNode())
