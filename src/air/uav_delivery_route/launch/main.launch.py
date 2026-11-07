#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动在线调度节点（订阅 odom/电量，发布下一目标站与换电建议）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_delivery_route")
    params = os.path.join(share, "config", "uav_delivery_route.yaml")
    return LaunchDescription([
        Node(
            package="uav_delivery_route",
            executable="dispatch",
            name="uav_delivery_dispatch",
            output="screen",
            parameters=[params],
        ),
    ])
