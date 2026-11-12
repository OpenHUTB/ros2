#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动文件：启动多帧图像端到端策略仿真节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_rate_arg = DeclareLaunchArgument(
        "sim_rate", default_value="20.0", description="仿真控制频率 Hz")

    e2e_node = Node(
        package="uav_e2e_multiframe_image_policy",
        executable="e2e_node",
        name="uav_e2e_multiframe_image_policy",
        output="screen",
        parameters=[{"sim_rate": 20.0}],
    )

    return LaunchDescription([sim_rate_arg, e2e_node])
