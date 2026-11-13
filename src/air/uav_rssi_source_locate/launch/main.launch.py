#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the scan node with config/uav_rssi_source_locate.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_rssi_source_locate")
    params = os.path.join(share, "config", "uav_rssi_source_locate.yaml")
    return LaunchDescription([
        Node(
            package="uav_rssi_source_locate",
            executable="scan_node",
            name="uav_rssi_source_locate_node",
            output="screen",
            parameters=[params],
        ),
    ])
