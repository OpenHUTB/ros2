#!/usr/bin/env python3
"""ROS 2 node: smooth a global Path with Catmull-Rom splines."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

try:
    from .planner import catmull_rom
except ImportError:
    from planner import catmull_rom


class SmoothPathNode(Node):
    def __init__(self) -> None:
        super().__init__("path_smoothing")
        self.declare_parameter("input_topic", "/plan")
        self.declare_parameter("output_topic", "/plan_smoothed")
        self.declare_parameter("samples", 12)
        self.samples = int(self.get_parameter("samples").value)
        self.pub = self.create_publisher(
            Path, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Path, str(self.get_parameter("input_topic").value), self.on_path, 10)
        self.get_logger().info("path_smoothing ready")

    def on_path(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) < 2:
            self.pub.publish(msg)
            return
        smooth = catmull_rom(points, samples=self.samples)
        out = Path()
        out.header = msg.header
        for i, (x, y) in enumerate(smooth):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            j = min(i + 1, len(smooth) - 1)
            k = max(0, i - 1)
            yaw = math.atan2(smooth[j][1] - smooth[k][1],
                             smooth[j][0] - smooth[k][0])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            out.poses.append(pose)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SmoothPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
