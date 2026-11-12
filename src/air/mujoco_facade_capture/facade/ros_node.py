"""以 ROS 2 话题驱动的 MuJoCo 幕墙扫描任务节点。"""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMessage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from .learning import DenseController, control_features
from .route import scan_route
from .scene import ray_features
from .simulation import FacadeSimulation, expert_velocity


class FacadeMissionNode(Node):
    """以 100 Hz 控制周期推进仿真，并发布传感与任务状态。"""

    def __init__(self) -> None:
        super().__init__("mujoco_facade_mission")
        self.declare_parameter("style", "grid")
        self.declare_parameter("distance", 4.0)
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("duration", 70.0)
        self.declare_parameter("controller", "neural")
        share = Path(get_package_share_directory("mujoco_facade_capture"))
        self.declare_parameter("model", str(share / "models/facade_controller.json"))
        self.declare_parameter("output", "/tmp/facade_scan")
        self.declare_parameter("override_timeout", 0.5)

        self.style = str(self.get_parameter("style").value)
        self.distance = float(self.get_parameter("distance").value)
        self.duration = float(self.get_parameter("duration").value)
        if self.duration <= 0 or not math.isfinite(self.duration):
            raise ValueError("duration 必须为正的有限数")
        self.controller_name = str(self.get_parameter("controller").value)
        if self.controller_name not in {"neural", "proportional"}:
            raise ValueError("controller 必须是 neural 或 proportional")
        self.override_timeout = float(self.get_parameter("override_timeout").value)
        self.output = Path(str(self.get_parameter("output").value)).expanduser()
        self.output.mkdir(parents=True, exist_ok=True)
        self.model = None
        if self.controller_name == "neural":
            self.model = DenseController.load(
                Path(str(self.get_parameter("model").value))
            )

        self.sim = FacadeSimulation(
            self.style, self.distance, float(self.get_parameter("offset").value)
        )
        self.control_period = 0.01
        physics_period = float(self.sim.model.opt.timestep)
        step_ratio = self.control_period / physics_period
        self.physics_steps_per_tick = round(step_ratio)
        if self.physics_steps_per_tick < 1 or not math.isclose(
            step_ratio, self.physics_steps_per_tick, rel_tol=0.0, abs_tol=1e-9
        ):
            self.sim.close()
            raise ValueError("MuJoCo 时间步必须整除 0.01 秒 ROS 控制周期")
        self.route = scan_route(distance=self.distance)
        self.waypoint = 0
        self.steps = 0
        self.control_ticks = 0
        self.rows: list[list[float]] = []
        self.distances: list[float] = []
        self.override: np.ndarray | None = None
        self.override_time = 0.0
        self.completed = False
        self._last_status = ""

        self.image_pub = self.create_publisher(ImageMessage, "/facade/image", 10)
        self.range_pub = self.create_publisher(Float32MultiArray, "/facade/range", 10)
        self.command_pub = self.create_publisher(Twist, "/facade/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/facade/status", 10)
        self.override_sub = self.create_subscription(
            Twist, "/facade/cmd_vel_override", self._on_override, 10
        )
        self.timer = self.create_timer(self.control_period, self._tick)
        self._publish_status(f"running:0/{len(self.route)}")
        self.get_logger().info(
            f"启动{self.style}幕墙扫描，控制器={self.controller_name}，航点={len(self.route)}"
        )

    def _on_override(self, message: Twist) -> None:
        values = np.asarray(
            [message.linear.x, message.linear.y, message.linear.z], dtype=float
        )
        if not np.isfinite(values).all():
            self.get_logger().warning("忽略包含非有限数值的速度覆盖指令")
            return
        changed = self.override is None or not np.allclose(values, self.override)
        self.override = np.clip(values, -1.0, 1.0)
        self.override_time = time.monotonic()
        if changed:
            self.get_logger().info(
                f"收到外部速度覆盖：{self.override[0]:.2f}, {self.override[1]:.2f}, {self.override[2]:.2f} m/s"
            )

    def _publish_range(self, values: np.ndarray) -> None:
        message = Float32MultiArray()
        message.layout.dim = [
            MultiArrayDimension(label="nine_ray_distance_m", size=9, stride=9)
        ]
        message.data = (
            np.nan_to_num(values, nan=0.0, posinf=0.0).astype(np.float32).tolist()
        )
        self.range_pub.publish(message)

    def _publish_command(self, command: np.ndarray) -> None:
        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = map(float, command)
        self.command_pub.publish(message)

    def _publish_image(self) -> None:
        pixels = self.sim.image(width=320, height=240)
        message = ImageMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "facade_camera"
        message.height, message.width = pixels.shape[:2]
        message.encoding, message.is_bigendian = "rgb8", 0
        message.step = int(pixels.shape[1] * 3)
        message.data = pixels.tobytes()
        self.image_pub.publish(message)

    def _tick(self) -> None:
        ranges = self.sim.ranges()
        projected = ray_features(ranges)
        valid = projected[np.isfinite(projected)]
        measured_distance = float(np.median(valid)) if valid.size else math.nan
        self._publish_range(ranges)
        if self.completed:
            self._publish_command(np.zeros(3))
            return

        target = self.route[self.waypoint]
        error = target - self.sim.position
        if math.isfinite(measured_distance):
            error[1] = measured_distance - self.distance
        velocity = self.sim.velocity
        if (
            self.override is not None
            and time.monotonic() - self.override_time < self.override_timeout
        ):
            command = self.override.copy()
        elif not math.isfinite(measured_distance):
            command = np.zeros(3)
        elif self.model is None:
            command = expert_velocity(error)
        else:
            features = control_features(error, ranges, velocity, self.distance)
            command = self.model.predict(features)
        for _ in range(self.physics_steps_per_tick):
            self.sim.step(command)
        self._publish_command(command)
        self.steps += self.physics_steps_per_tick
        self.control_ticks += 1
        if self.control_ticks % 10 == 0:
            self.rows.append(
                [
                    self.sim.data.time,
                    *self.sim.position,
                    *self.sim.velocity,
                    measured_distance,
                    self.waypoint,
                ]
            )
            if math.isfinite(measured_distance):
                self.distances.append(measured_distance)
            self._publish_image()

        if (
            np.linalg.norm(target - self.sim.position) < 0.12
            and np.linalg.norm(self.sim.velocity) < 0.2
        ):
            PILImage.fromarray(self.sim.image()).save(
                self.output / f"capture_{self.waypoint:03d}.png"
            )
            self.waypoint += 1
            if self.waypoint >= len(self.route):
                self._finish()
            else:
                self._publish_status(f"scanning:{self.waypoint}/{len(self.route)}")
        elif self.sim.data.time >= self.duration:
            self._finish()

    def _publish_status(self, value: str) -> None:
        if value != self._last_status:
            message = String()
            message.data = value
            self.status_pub.publish(message)
            self._last_status = value

    def _finish(self) -> None:
        if self.completed:
            return
        self.completed = True
        PILImage.fromarray(self.sim.image("overview")).save(
            self.output / "overview.png"
        )
        with (self.output / "trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "time_s",
                    "x_m",
                    "y_m",
                    "z_m",
                    "vx_m_s",
                    "vy_m_s",
                    "vz_m_s",
                    "distance_m",
                    "waypoint",
                ]
            )
            writer.writerows(self.rows)
        distances = np.asarray(self.distances, dtype=float)
        metrics = {
            "style": self.style,
            "controller": self.controller_name,
            "waypoints_total": len(self.route),
            "waypoints_completed": self.waypoint,
            "completion_ratio": self.waypoint / len(self.route),
            "duration_s": self.sim.data.time,
            "distance_rmse_m": float(
                np.sqrt(np.mean((distances - self.distance) ** 2))
                if distances.size
                else math.nan
            ),
            "minimum_wall_distance_m": float(
                np.min(distances) if distances.size else math.nan
            ),
            "maximum_speed_m_s": float(
                max((np.linalg.norm(row[4:7]) for row in self.rows), default=0.0)
            ),
        }
        (self.output / "metrics.json").write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        status = "complete" if self.waypoint == len(self.route) else "time_limit"
        self._publish_status(status)
        self.get_logger().info(
            f"扫描结束：状态={status}，航点={self.waypoint}/{len(self.route)}，结果位于 {self.output}"
        )

    def destroy_node(self):
        self.sim.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FacadeMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
