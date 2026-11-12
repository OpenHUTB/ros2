"""ROS 2 nodes: reference publisher and MPC tracker."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from .core import MpcConfig
from .scene import ReferencePath, TrackConfig, run_mpc


class MpcReference(Node):
    """Publish the S-curve reference as ``nav_msgs/Path`` and the scripted run."""

    def __init__(self) -> None:
        super().__init__("mpc_reference")
        self.declare_parameter("duration_s", 20.0)
        self.declare_parameter("offset_m", 1.2)
        self.duration_s = max(1.0, float(self.get_parameter("duration_s").value))
        self.path = ReferencePath()
        self.track = TrackConfig(
            duration_s=self.duration_s,
            lateral_offset=float(self.get_parameter("offset_m").value),
        )
        self.run = run_mpc(self.path, self.track, MpcConfig())
        self.path_pub = self.create_publisher(Path, "/mpc/reference", 10)
        self.state_pub = self.create_publisher(PoseStamped, "/mpc/state", 20)
        self.status_pub = self.create_publisher(String, "/mpc/status", 10)
        self.index = 0
        self.publish_reference()
        self.create_timer(1.0 / self.track.control_hz, self.step)
        metrics = {
            "steps": int(len(self.run["states"])),
            "duration_s": self.duration_s,
            "offset_m": self.track.lateral_offset,
        }
        self.status_pub.publish(String(data=json.dumps(metrics)))
        self.get_logger().info(f"mpc_reference ready: {metrics}")

    def publish_reference(self) -> None:
        message = Path()
        message.header.frame_id = "map"
        for index in range(0, len(self.path.x), 25):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(self.path.x[index])
            pose.pose.position.y = float(self.path.y[index])
            message.poses.append(pose)
        self.path_pub.publish(message)

    def step(self) -> None:
        index = self.index % len(self.run["states"])
        state = self.run["states"][index]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(state[0])
        pose.pose.position.y = float(state[1])
        pose.pose.orientation.z = float(np.sin(state[2] / 2.0))
        pose.pose.orientation.w = float(np.cos(state[2] / 2.0))
        self.state_pub.publish(pose)
        self.index += 1


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_mpc_reference(args=None) -> None:
    rclpy.init(args=args)
    _spin(MpcReference())


def main_mpc_tracker(args=None) -> None:
    rclpy.init(args=args)
    _spin(MpcReference())
