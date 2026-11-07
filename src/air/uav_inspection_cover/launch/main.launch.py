#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动在线巡检进度节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_inspection_cover")
    params = os.path.join(share, "config", "inspect.yaml")
    return LaunchDescription([
        Node(
            package="uav_inspection_cover",
            executable="inspect",
            name="uav_inspection_cover_node",
            output="screen",
            parameters=[params],
        ),
    ])
