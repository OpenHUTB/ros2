#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动在线间隔监视节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_deconflict")
    params = os.path.join(share, "config", "params.yaml")
    return LaunchDescription([
        Node(
            package="uav_deconflict",
            executable="monitor",
            name="uav_deconflict_monitor",
            output="screen",
            parameters=[params],
        ),
    ])
