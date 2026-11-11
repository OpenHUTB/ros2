#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_e2e_rl 部署 launch（ROS2 Humble）。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("uav_e2e_rl")
    return LaunchDescription([
        Node(
            package="uav_e2e_rl",
            executable="deploy",
            name="e2e_rl_deploy",
            output="screen",
            parameters=[os.path.join(share, "config", "e2e.yaml")],
        ),
    ])
