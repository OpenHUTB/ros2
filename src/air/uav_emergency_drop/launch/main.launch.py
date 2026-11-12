#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the drop decision node with config/uav_emergency_drop.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_emergency_drop")
    params = os.path.join(share, "config", "uav_emergency_drop.yaml")
    return LaunchDescription([
        Node(
            package="uav_emergency_drop",
            executable="drop_node",
            name="uav_emergency_drop_node",
            output="screen",
            parameters=[params],
        ),
    ])
