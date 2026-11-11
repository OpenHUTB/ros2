#!/usr/bin/env python3
"""ROS 2 node: publish (v, w) speed profile for a Path."""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    from .profile import speed_profile
except ImportError:
    from profile import speed_profile


class VelocityProfileNode(Node):
    def __init__(self) -> None:
        super().__init__("velocity_profiling")
        self.declare_parameter("input_topic", "/plan")
        self.declare_parameter("output_topic", "/cmd_profile")
        self.declare_parameter("v_max", 1.0)
        self.declare_parameter("a_max", 0.8)
        self.declare_parameter("a_lat_max", 0.5)
        self.declare_parameter("min_speed", 0.12)
        self.pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Path, str(self.get_parameter("input_topic").value), self.on_path, 10)
        self.get_logger().info("velocity_profiling ready")

    def on_path(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) < 3:
            return
        profile = speed_profile(
            points,
            v_max=float(self.get_parameter("v_max").value),
            a_max=float(self.get_parameter("a_max").value),
            a_lat_max=float(self.get_parameter("a_lat_max").value),
            min_speed=float(self.get_parameter("min_speed").value),
        )
        w = profile.curvature * profile.speed
        data = np.column_stack([profile.speed, w]).ravel().tolist()
        out = Float64MultiArray()
        out.data = data
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = VelocityProfileNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
