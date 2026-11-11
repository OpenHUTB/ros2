"""Consume scan and odometry topics, publish a path, and record run metrics."""

from __future__ import annotations

import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as OccupancyGridMessage
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .navigation import GridNavigator


class RoundaboutNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("roundabout_navigation")
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("metrics_path", "/tmp/roundabout_navigation_metrics.json")
        self.navigator = GridNavigator(
            goal=(float(self.get_parameter("goal_x").value), float(self.get_parameter("goal_y").value))
        )
        self.metrics_path = str(self.get_parameter("metrics_path").value)
        self.pose: tuple[float, float, float] | None = None
        self.last_stamp = 0.0
        self.simulation_done = False
        self.metrics_saved = False
        self.finalize_timer = None
        self.map_pub = self.create_publisher(OccupancyGridMessage, "/map", 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.metrics_pub = self.create_publisher(String, "/slam_navigation/metrics", 10)
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 20)
        self.create_subscription(Bool, "/simulation/collision", self._on_collision, 20)
        self.create_subscription(Bool, "/simulation/route_complete", self._on_route_complete, 20)
        self.create_subscription(Bool, "/simulation/done", self._on_done, 20)

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.pose = (position.x, position.y, yaw)
        self.last_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def _on_scan(self, message: LaserScan) -> None:
        if self.pose is None:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        result = self.navigator.update_scan(
            self.pose,
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_max,
            stamp,
        )
        self._publish_map(message.header)
        self._publish_path(message.header, result.path)
        self._publish_metrics()

    def _on_collision(self, message: Bool) -> None:
        self.navigator.record_collision(message.data)

    def _on_route_complete(self, message: Bool) -> None:
        if message.data:
            self.navigator.finish_episode(True)

    def _on_done(self, message: Bool) -> None:
        if message.data and not self.simulation_done:
            self.simulation_done = True
            self.finalize_timer = self.create_timer(0.2, self._finalize_metrics)

    def _finalize_metrics(self) -> None:
        if self.finalize_timer is not None:
            self.finalize_timer.cancel()
        self._save_metrics()

    def _publish_path(self, header, cells) -> None:
        message = Path()
        message.header = header
        for x, y in cells:
            world_x, world_y = self.navigator.cell_to_world((x, y))
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_map(self, header) -> None:
        grid = self.navigator.grid
        message = OccupancyGridMessage()
        message.header = header
        message.header.frame_id = "map"
        message.info.resolution = self.navigator.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = self.navigator.origin[0]
        message.info.origin.position.y = self.navigator.origin[1]
        message.info.origin.orientation.w = 1.0
        cell_values = {
            grid.UNKNOWN: -1,
            grid.FREE: 0,
            grid.OCCUPIED: 100,
        }
        message.data = [cell_values[cell] for row in grid.cells for cell in row]
        self.map_pub.publish(message)

    def _publish_metrics(self) -> None:
        payload = json.dumps(self.navigator.metrics(), ensure_ascii=False, sort_keys=True)
        self.metrics_pub.publish(String(data=payload))

    def _save_metrics(self) -> None:
        if self.metrics_saved:
            return
        payload = self.navigator.metrics()
        folder = os.path.dirname(self.metrics_path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        with open(self.metrics_path, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
        self.metrics_saved = True
        self.get_logger().info(f"navigation metrics saved: {self.metrics_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoundaboutNavigationNode()
    try:
        while rclpy.ok() and not node.metrics_saved:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node.navigator.update_count and not node.metrics_saved:
            node._save_metrics()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
