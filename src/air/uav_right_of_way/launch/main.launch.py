#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动通行权在线决策节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_right_of_way")
    params = os.path.join(share, "config", "uav_right_of_way.yaml")
    return LaunchDescription([
        Node(
            package="uav_right_of_way",
            executable="row_node",
            name="uav_right_of_way_node",
            output="screen",
            parameters=[params],
        ),
    ])
