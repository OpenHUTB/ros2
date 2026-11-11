#!/usr/bin/env python3
"""ROS 2 node: publish line and door features from /scan."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

try:
    from .features import detect_doors, extract_lines, points_from_scan
except ImportError:
    from features import detect_doors, extract_lines, points_from_scan


class LidarFeaturesNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_features")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_topic", "/lidar_features")
        self.declare_parameter("split_threshold", 0.08)
        self.declare_parameter("min_points", 4)
        self.declare_parameter("max_gap", 0.5)
        self.pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan,
            qos_profile_sensor_data)
        self.get_logger().info("lidar_features ready")

    def on_scan(self, msg: LaserScan) -> None:
        points = points_from_scan(msg.ranges, msg.angle_min,
                                  msg.angle_increment, msg.range_max,
                                  msg.range_min)
        lines = extract_lines(points,
                              threshold=float(self.get_parameter("split_threshold").value),
                              min_points=int(self.get_parameter("min_points").value),
                              max_gap=float(self.get_parameter("max_gap").value),
                              angle_increment=msg.angle_increment)
        doors = detect_doors(lines)
        markers = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, line in enumerate(lines):
            m = Marker()
            m.header = msg.header
            m.ns = "lines"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color.b, m.color.g, m.color.r, m.color.a = 1.0, 0.5, 0.2, 1.0
            for x, y in (line.p1, line.p2):
                p = Point()
                p.x, p.y = float(x), float(y)
                m.points.append(p)
            markers.markers.append(m)
        for i, (x1, y1, x2, y2) in enumerate(doors):
            m = Marker()
            m.header = msg.header
            m.ns = "doors"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.07
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 1.0
            for x, y in ((x1, y1), (x2, y2)):
                p = Point()
                p.x, p.y = float(x), float(y)
                m.points.append(p)
            markers.markers.append(m)
        self.pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = LidarFeaturesNode()
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

