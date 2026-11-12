#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动文件：启动下视循迹神经网络仿真节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_rate_arg = DeclareLaunchArgument(
        "sim_rate", default_value="50.0", description="仿真控制频率 Hz")

    track_node = Node(
        package="uav_downview_centerline_track_nn",
        executable="track_node",
        name="uav_downview_centerline_track_nn",
        output="screen",
        parameters=[{"sim_rate": 50.0}],
    )

    return LaunchDescription([sim_rate_arg, track_node])
