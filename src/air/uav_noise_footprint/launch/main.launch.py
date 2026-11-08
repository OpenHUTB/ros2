#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动在线噪声告警节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_noise_footprint")
    params = os.path.join(share, "config", "params.yaml")
    return LaunchDescription([
        Node(
            package="uav_noise_footprint",
            executable="noise_monitor",
            name="uav_noise_monitor",
            output="screen",
            parameters=[params],
        ),
    ])
