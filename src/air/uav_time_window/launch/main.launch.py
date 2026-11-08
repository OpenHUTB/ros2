#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动在线调度节点（订阅各机里程计，周期发布任务分配表）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_time_window")
    params = os.path.join(share, "config", "uav_time_window.yaml")
    return LaunchDescription([
        Node(
            package="uav_time_window",
            executable="schedule",
            name="uav_time_window_schedule",
            output="screen",
            parameters=[params],
        ),
    ])
