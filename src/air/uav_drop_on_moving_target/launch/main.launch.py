#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the drop decision node with config/uav_drop_on_moving_target.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_drop_on_moving_target")
    params = os.path.join(share, "config", "uav_drop_on_moving_target.yaml")
    return LaunchDescription([
        Node(
            package="uav_drop_on_moving_target",
            executable="drop_node",
            name="uav_drop_on_moving_target_node",
            output="screen",
            parameters=[params],
        ),
    ])
