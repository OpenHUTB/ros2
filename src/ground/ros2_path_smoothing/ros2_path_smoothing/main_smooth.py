#!/usr/bin/env python3
"""ROS 2 node: simplify and smooth a Path on a map, with safety fallback."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from .planner import Grid, smooth_path
except ImportError:
    from planner import Grid, smooth_path


class SmoothPathNode(Node):
    def __init__(self) -> None:
        super().__init__("path_smoothing")
        self.declare_parameter("input_topic", "/plan")
        self.declare_parameter("output_topic", "/plan_smoothed")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("min_clearance", 0.1)
        self.min_clearance = float(self.get_parameter("min_clearance").value)
        self.grid = None
        self.last_path = None
        qos_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            Path, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Path, str(self.get_parameter("input_topic").value), self.on_path, 10)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value),
            self.on_map, qos_map)
        self.get_logger().info("path_smoothing ready (waiting for /map)")

    def on_map(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                             msg.info.width)
        occupied = (data >= 50) | (data < 0)
        self.grid = Grid(occupied, msg.info.resolution,
                         (msg.info.origin.position.x, msg.info.origin.position.y))
        if self.last_path is not None:
            self.on_path(self.last_path)

    def on_path(self, msg: Path) -> None:
        self.last_path = msg
        if self.grid is None:
            self.get_logger().warn("no /map yet, publishing input path unchanged")
            self.pub.publish(msg)
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) < 2:
            self.pub.publish(msg)
            return
        cells = [self.grid.to_cell(p) for p in points]
        smoothed, safe = smooth_path(self.grid, cells, self.min_clearance)
        if not safe:
            self.get_logger().warn("smoothed path below clearance, publishing input")
            self.pub.publish(msg)
            return
        out = Path()
        out.header = msg.header
        for i, (x, y) in enumerate(smoothed):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            j = min(i + 1, len(smoothed) - 1)
            k = max(0, i - 1)
            yaw = math.atan2(smoothed[j][1] - smoothed[k][1],
                             smoothed[j][0] - smoothed[k][0])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            out.poses.append(pose)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SmoothPathNode()
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
