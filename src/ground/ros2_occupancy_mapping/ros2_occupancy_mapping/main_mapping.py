#!/usr/bin/env python3
"""ROS 2 node: log-odds occupancy grid from /scan + /odom."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan

try:
    from .mapping import LogOddsGrid
except ImportError:
    from mapping import LogOddsGrid


class MappingNode(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_mapping")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("x_min", -10.0)
        self.declare_parameter("x_max", 10.0)
        self.declare_parameter("y_min", -10.0)
        self.declare_parameter("y_max", 10.0)

        self.grid = LogOddsGrid(
            float(self.get_parameter("x_min").value),
            float(self.get_parameter("y_min").value),
            float(self.get_parameter("x_max").value),
            float(self.get_parameter("y_max").value),
            float(self.get_parameter("resolution").value),
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.pose = None
        self.scan = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("map_topic").value), qos)
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10)
        self.get_logger().info(
            f"occupancy_mapping ready: {self.grid.width}x{self.grid.height} cells")

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_scan(self, msg: LaserScan) -> None:
        if self.pose is None:
            return
        self.grid.update_scan(self.pose, msg.ranges, msg.angle_min,
                              msg.angle_increment, msg.range_max)
        self.publish(msg.header.stamp)

    def publish(self, stamp) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.x_min
        msg.info.origin.position.y = self.grid.y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.as_occupancy().ravel().tolist()
        self.map_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MappingNode()
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

