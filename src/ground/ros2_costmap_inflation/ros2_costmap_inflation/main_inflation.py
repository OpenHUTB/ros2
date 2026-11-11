#!/usr/bin/env python3
"""ROS 2 node: inflate an OccupancyGrid into a cost map."""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from .inflation import inflate
except ImportError:
    from inflation import inflate


class InflationNode(Node):
    def __init__(self) -> None:
        super().__init__("costmap_inflation")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/costmap")
        self.declare_parameter("inflation_radius", 0.5)
        self.declare_parameter("inscribed_radius", 0.2)
        self.declare_parameter("cost_scaling", 3.0)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("costmap_topic").value), qos)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value), self.on_map, qos)
        self.get_logger().info("costmap_inflation ready")

    def on_map(self, msg: OccupancyGrid) -> None:
        expected = msg.info.height * msg.info.width
        if len(msg.data) != expected:
            self.get_logger().error(f"map data {len(msg.data)} != {expected}, skip")
            return
        occ = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                           msg.info.width)
        result = inflate(
            occ, resolution=msg.info.resolution,
            inflation_radius=float(self.get_parameter("inflation_radius").value),
            inscribed_radius=float(self.get_parameter("inscribed_radius").value),
            cost_scaling=float(self.get_parameter("cost_scaling").value),
        )
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = result.costs.ravel().tolist()
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = InflationNode()
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

