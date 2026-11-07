#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发布静态占用栅格地图（transient local，晚启动的节点也能收到）。"""

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from diff_drive_astar_odom_nav.map_utils import load_map


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__("static_map_publisher")
        default_map = str(Path(get_package_share_directory(
            "diff_drive_astar_odom_nav")) / "maps" / "static_map.yaml")
        self.declare_parameter("map_yaml", default_map)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "map")

        self.grid_map = load_map(self.get_parameter("map_yaml").value)
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("map_topic").value, qos)
        self.msg = self._build_msg()
        self.pub.publish(self.msg)
        self.get_logger().info(
            "静态地图已发布：%d x %d，分辨率 %.3f m，占用格 %d"
            % (self.grid_map.width, self.grid_map.height,
               self.grid_map.resolution, int(self.grid_map.grid.sum())))

    def _build_msg(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid_map.resolution
        msg.info.width = self.grid_map.width
        msg.info.height = self.grid_map.height
        msg.info.origin.position.x = self.grid_map.origin_x
        msg.info.origin.position.y = self.grid_map.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid_map.occupancy_values()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
