"""ROS 2 nodes for motor thermal guard."""

from __future__ import annotations

from dataclasses import asdict
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import MotorSample, MotorTempGuard
from .scenario import scenario_at


class MotorSim(Node):
    def __init__(self) -> None:
        super().__init__("uav_motor_sim")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(String, "/uav_motor/sample", 10)
        self.create_timer(1.0 / self.rate_hz, self.pub)

    def pub(self) -> None:
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        s = scenario_at(elapsed)
        self.pub.publish(String(data=json.dumps(asdict(s), ensure_ascii=False)))
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            rclpy.shutdown()


class MotorTempNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_motor_temp_guard")
        self.declare_parameter("output_csv", "/tmp/uav_motor_temp.csv")
        self.guard = MotorTempGuard()
        self.pub = self.create_publisher(String, "/uav_motor/assessment", 10)
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.f = self.csv_path.open("w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "state", "temp_c", "current_a"])
        self.f.flush()
        self.last = None
        self.create_subscription(String, "/uav_motor/sample", self.on_s, 10)

    def on_s(self, msg: String) -> None:
        try:
            s = MotorSample(**json.loads(msg.data))
            a = self.guard.evaluate(s)
        except Exception as exc:
            self.get_logger().warning(f"bad: {exc}")
            return
        self.pub.publish(String(data=json.dumps(a.to_dict(), ensure_ascii=False)))
        self.w.writerow([f"{s.timestamp_s:.2f}", a.state, f"{a.estimated_temp_c:.1f}", f"{s.current_a:.1f}"])
        self.f.flush()
        if a.state != self.last:
            self.get_logger().info(f"state={a.state} T={a.estimated_temp_c:.1f}C -> {a.recommendation}")
            self.last = a.state

    def destroy_node(self):
        self.f.close()
        return super().destroy_node()


def _spin(n):
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
    _spin(MotorSim())


def main_guard():
    rclpy.init()
    _spin(MotorTempNode())
