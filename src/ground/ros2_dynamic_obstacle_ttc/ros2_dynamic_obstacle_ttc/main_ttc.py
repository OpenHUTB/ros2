#!/usr/bin/env python3
"""ROS 2 node: dynamic obstacle tracking and TTC warning."""

from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

try:
    from .ttc_core import Tracker, WarningGate, cluster_scan, time_to_collision
except ImportError:
    from ttc_core import Tracker, WarningGate, cluster_scan, time_to_collision


class TtcNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_ttc")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("warning_topic", "/collision_warning")
        self.declare_parameter("marker_topic", "/moving_obstacles")
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("max_ttc", 10.0)
        self.declare_parameter("on_ttc", 2.5)
        self.declare_parameter("off_ttc", 3.5)
        self.declare_parameter("robot_radius", 0.25)
        self.declare_parameter("safety_margin", 0.10)

        self.max_range = float(self.get_parameter("max_range").value)
        self.max_ttc = float(self.get_parameter("max_ttc").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)
        self.gate = WarningGate(
            on_ttc=float(self.get_parameter("on_ttc").value),
            off_ttc=float(self.get_parameter("off_ttc").value),
        )
        self.tracker = Tracker()
        self.warning_pub = self.create_publisher(
            Bool, str(self.get_parameter("warning_topic").value), 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan,
            qos_profile_sensor_data)
        self._last_warning = False
        self._last_log = 0.0
        self.get_logger().info("dynamic_obstacle_ttc ready")

    def on_scan(self, msg: LaserScan) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        clusters = cluster_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                max_range=self.max_range)
        tracks = self.tracker.update(clusters, t)
        best = min((time_to_collision(tr, self.robot_radius, self.safety_margin,
                                      self.max_ttc) for tr in tracks),
                   default=float("inf"))
        active = self.gate.update(best)
        self.warning_pub.publish(Bool(data=active))

        markers = MarkerArray()
        for track in tracks:
            marker = Marker()
            marker.header = msg.header
            marker.header.frame_id = msg.header.frame_id or "base_link"
            marker.ns = "moving_obstacles"
            marker.id = track.track_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = Point(x=track.x, y=track.y, z=0.3)
            marker.scale.x = marker.scale.y = max(0.2, 2.0 * track.radius)
            marker.scale.z = 0.6
            marker.color.r = 1.0 if active else 0.9
            marker.color.g = 0.2
            marker.color.a = 0.7
            marker.lifetime = Duration(sec=0, nanosec=500_000_000)
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

        now = time.monotonic()
        if active != self._last_warning or now - self._last_log > 2.0:
            self._last_warning = active
            self._last_log = now
            state = "WARN" if active else "clear"
            ttc = f"{best:.2f}s" if math.isfinite(best) else "inf"
            self.get_logger().info(
                f"{state}: tracks={len(tracks)} min_ttc={ttc}")


def main() -> None:
    rclpy.init()
    node = TtcNode()
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


