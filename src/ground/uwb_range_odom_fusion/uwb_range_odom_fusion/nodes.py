"""ROS 2 nodes: a deterministic UWB/odometry simulator and the fusion node.

The simulator replays exactly the same deterministic run as the offline
benchmark, so ``ros2 launch`` and ``python main.py`` agree sample by sample.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .ekf import RangeEkf
from .odometry import unicycle_step
from .scenario import ScenarioConfig, simulate


def _quaternion_from_yaw(yaw: float):
    return 0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))


class UwbSimulator(Node):
    """Publish ground truth, wheel odometry and UWB ranges on a timer."""

    def __init__(self) -> None:
        super().__init__("uwb_sim")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("duration_s", 0.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("speed", 0.45)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.seed = int(self.get_parameter("seed").value)
        speed = float(self.get_parameter("speed").value)

        config = ScenarioConfig(
            control_hz=self.rate_hz,
            truth_hz=self.rate_hz * 10.0,
            duration_s=max(70.0, self.duration_s),
            speed=speed,
        )
        self.simulation = simulate(config, seed=self.seed)
        self.config = self.simulation.config

        self.truth_pub = self.create_publisher(PoseStamped, "/ground_truth/pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.range_pubs = [
            self.create_publisher(Range, f"/uwb/anchor_{index}/range", 10)
            for index in range(len(self.config.anchor_array))
        ]
        self.epoch = 0
        self.started = self.get_clock().now()
        self.odometry_pose = self.simulation.truth[0].copy()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)
        self.get_logger().info(
            f"uwb_sim ready: {len(self.config.anchor_array)} anchors, "
            f"{len(self.simulation.truth)} epochs, seed {self.seed}"
        )

    def step(self) -> None:
        index = self.epoch
        if index >= len(self.simulation.truth):
            if self.duration_s > 0.0:
                self.get_logger().info("simulation finished")
                rclpy.shutdown()
            else:
                self.epoch = 0
            return
        stamp = self.get_clock().now().to_msg()
        truth = self.simulation.truth[index]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position.x = float(truth[0])
        pose.pose.position.y = float(truth[1])
        quaternion = _quaternion_from_yaw(float(truth[2]))
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.truth_pub.publish(pose)

        odometry = Odometry()
        odometry.header.frame_id = "map"
        odometry.header.stamp = stamp
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = float(self.odometry_pose[0])
        odometry.pose.pose.position.y = float(self.odometry_pose[1])
        yaw = _quaternion_from_yaw(float(self.odometry_pose[2]))
        odometry.pose.pose.orientation.z = yaw[2]
        odometry.pose.pose.orientation.w = yaw[3]
        odometry.twist.twist.linear.x = float(self.simulation.odometry[index, 0])
        odometry.twist.twist.angular.z = float(self.simulation.odometry[index, 1])
        self.odom_pub.publish(odometry)
        self.odometry_pose = np.array(
            unicycle_step(
                self.odometry_pose,
                self.simulation.odometry[index, 0],
                self.simulation.odometry[index, 1],
                1.0 / self.rate_hz,
            )
        )

        for anchor_index, publisher in enumerate(self.range_pubs):
            measured = self.simulation.ranges[index, anchor_index]
            if not np.isfinite(measured):
                continue
            message = Range()
            message.header.frame_id = "map"
            message.header.stamp = stamp
            message.radiation_type = Range.ULTRASOUND
            message.field_of_view = float(2.0 * np.pi)
            message.min_range = 0.05
            message.max_range = 60.0
            message.range = float(measured)
            publisher.publish(message)
        self.epoch += 1


class UwbFusion(Node):
    """Predict with wheel odometry, update with gated UWB ranges."""

    def __init__(self) -> None:
        super().__init__("uwb_fusion")
        self.declare_parameter("anchor_x", [6.0, -6.0, -6.0, 6.0])
        self.declare_parameter("anchor_y", [4.0, 4.0, -4.0, -4.0])
        self.declare_parameter("range_sigma", 0.04)
        self.declare_parameter("gate_chi2", 6.635)
        self.declare_parameter("use_gating", True)
        self.declare_parameter("process_sigma_v", 0.05)
        self.declare_parameter("process_sigma_omega", 0.07)
        self.declare_parameter("output_csv", "/tmp/uwb_range_odom_fusion.csv")

        anchor_x = [float(value) for value in self.get_parameter("anchor_x").value]
        anchor_y = [float(value) for value in self.get_parameter("anchor_y").value]
        if len(anchor_x) != len(anchor_y) or len(anchor_x) < 3:
            raise ValueError("anchor_x and anchor_y must have the same length >= 3")
        self.anchors = np.column_stack([anchor_x, anchor_y])
        self.sigma_range = float(self.get_parameter("range_sigma").value)
        self.filter = RangeEkf(
            initial_state=np.zeros(3),
            initial_covariance=np.diag([0.5**2, 0.5**2, np.deg2rad(10.0) ** 2]),
            sigma_range=self.sigma_range,
            sigma_v=float(self.get_parameter("process_sigma_v").value),
            sigma_omega=float(self.get_parameter("process_sigma_omega").value),
            gate_chi2=float(self.get_parameter("gate_chi2").value),
            use_gating=bool(self.get_parameter("use_gating").value),
        )
        self.ranges = np.full(len(self.anchors), np.nan)
        self.initialised = False
        self.last_stamp = None
        self.accepted_total = 0
        self.rejected_total = 0

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/uwb/pose", 10)
        self.status_pub = self.create_publisher(String, "/uwb/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/uwb/markers", 10)
        self.create_subscription(Odometry, "/odom", self.on_odometry, 20)
        self.subscriptions_ranges = [
            self.create_subscription(
                Range,
                f"/uwb/anchor_{index}/range",
                lambda message, index=index: self.on_range(message, index),
                20,
            )
            for index in range(len(self.anchors))
        ]
        self.csv_path = Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_s", "x", "y", "theta", "accepted", "rejected"])
        self.create_timer(0.2, self.publish_markers)
        self.get_logger().info(
            f"uwb_fusion ready: {len(self.anchors)} anchors, gating="
            f"{self.filter.use_gating}, gate={self.filter.gate_chi2:.3f}"
        )

    def on_range(self, message: Range, index: int) -> None:
        self.ranges[index] = float(message.range)

    def on_odometry(self, message: Odometry) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not self.initialised:
            self.filter.state[0] = message.pose.pose.position.x
            self.filter.state[1] = message.pose.pose.position.y
            quaternion = message.pose.pose.orientation
            yaw = 2.0 * np.arctan2(quaternion.z, quaternion.w)
            self.filter.state[2] = float(yaw)
            self.initialised = True
            self.last_stamp = stamp
            return
        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if not 1e-4 < dt < 1.0:
            return
        self.filter.predict(
            message.twist.twist.linear.x, message.twist.twist.angular.z, dt
        )
        if np.any(np.isfinite(self.ranges)):
            outcome = self.filter.update(self.anchors, self.ranges)
            self.accepted_total += outcome.accepted
            self.rejected_total += outcome.rejected
            self.ranges[:] = np.nan
        self.publish_state(stamp)

    def publish_state(self, stamp: float) -> None:
        state = self.filter.state
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp.sec = int(stamp)
        pose.header.stamp.nanosec = int((stamp % 1.0) * 1e9)
        pose.pose.pose.position.x = float(state[0])
        pose.pose.pose.position.y = float(state[1])
        quaternion = _quaternion_from_yaw(float(state[2]))
        pose.pose.pose.orientation.z = quaternion[2]
        pose.pose.pose.orientation.w = quaternion[3]
        covariance = self.filter.covariance
        # PoseWithCovariance6x6 is [x, y, z, roll, pitch, yaw]: the state yaw
        # belongs in slot 5, not in slot 2 (which is the z translation).
        slots = (0, 1, 5)
        for row in range(3):
            for column in range(3):
                pose.pose.covariance[slots[row] * 6 + slots[column]] = float(
                    covariance[row, column]
                )
        self.pose_pub.publish(pose)

        status = {
            "x": float(state[0]),
            "y": float(state[1]),
            "theta": float(state[2]),
            "sigma_xy_m": float(np.sqrt(np.trace(covariance[:2, :2]))),
            "accepted_total": int(self.accepted_total),
            "rejected_total": int(self.rejected_total),
            "gating": bool(self.filter.use_gating),
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        self.writer.writerow(
            [
                f"{stamp:.3f}",
                f"{state[0]:.4f}",
                f"{state[1]:.4f}",
                f"{state[2]:.5f}",
                self.accepted_total,
                self.rejected_total,
            ]
        )
        self.csv_file.flush()

    def publish_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, anchor in enumerate(self.anchors):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "uwb_anchors"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(anchor[0])
            marker.pose.position.y = float(anchor[1])
            marker.pose.position.z = 0.4
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0
            markers.markers.append(marker)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "uwb_labels"
            label.id = 100 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(anchor[0])
            label.pose.position.y = float(anchor[1])
            label.pose.position.z = 0.9
            label.scale.z = 0.4
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"A{index}"
            markers.markers.append(label)

        estimate = Marker()
        estimate.header.frame_id = "map"
        estimate.header.stamp = stamp
        estimate.ns = "uwb_estimate"
        estimate.id = 200
        estimate.type = Marker.ARROW
        estimate.action = Marker.ADD
        estimate.pose.position.x = float(self.filter.state[0])
        estimate.pose.position.y = float(self.filter.state[1])
        estimate.pose.position.z = 0.25
        quaternion = _quaternion_from_yaw(float(self.filter.state[2]))
        estimate.pose.orientation.z = quaternion[2]
        estimate.pose.orientation.w = quaternion[3]
        estimate.scale.x = 0.8
        estimate.scale.y = 0.16
        estimate.scale.z = 0.16
        estimate.color.r = 0.11
        estimate.color.g = 0.52
        estimate.color.b = 0.29
        estimate.color.a = 1.0
        markers.markers.append(estimate)

        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = stamp
        text.ns = "uwb_status"
        text.id = 201
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(self.filter.state[0])
        text.pose.position.y = float(self.filter.state[1])
        text.pose.position.z = 1.1
        text.scale.z = 0.35
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"rejected={self.rejected_total} accepted={self.accepted_total}"
        markers.markers.append(text)
        self.marker_pub.publish(markers)

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
    _spin(UwbSimulator())


def main_fusion(args=None) -> None:
    rclpy.init(args=args)
    _spin(UwbFusion())
