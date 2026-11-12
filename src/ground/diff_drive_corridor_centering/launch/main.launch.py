#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动：虚拟通道测距仿真 + 双侧墙居中 PD 巡航控制器。

用法：
    ros2 launch diff_drive_corridor_centering main.launch.py

另开终端观测：
    ros2 topic echo /corridor/state
    ros2 topic echo /corridor/cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = "diff_drive_corridor_centering"
    share = get_package_share_directory(pkg)
    params_file = os.path.join(share, "config", "params.yaml")

    scan_sim = Node(
        package=pkg,
        executable="corridor_scan_sim",
        name="corridor_scan_sim",
        parameters=[params_file],
        output="screen",
    )
    centering = Node(
        package=pkg,
        executable="corridor_centering",
        name="corridor_centering",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([scan_sim, centering])
