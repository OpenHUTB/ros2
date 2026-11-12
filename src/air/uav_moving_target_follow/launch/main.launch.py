#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the follow node with config/uav_moving_target_follow.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_moving_target_follow")
    params = os.path.join(share, "config", "uav_moving_target_follow.yaml")
    return LaunchDescription([
        Node(
            package="uav_moving_target_follow",
            executable="follow_node",
            name="uav_moving_target_follow_node",
            output="screen",
            parameters=[params],
        ),
    ])
