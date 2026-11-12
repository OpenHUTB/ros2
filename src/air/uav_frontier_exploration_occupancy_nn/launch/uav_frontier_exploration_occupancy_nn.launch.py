#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动文件：启动占据栅格前沿探索神经网络仿真节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_rate_arg = DeclareLaunchArgument(
        "sim_rate", default_value="20.0", description="仿真控制频率 Hz")

    explorer_node = Node(
        package="uav_frontier_exploration_occupancy_nn",
        executable="explorer_node",
        name="uav_frontier_exploration_occupancy_nn",
        output="screen",
        parameters=[{"sim_rate": 20.0}],
    )

    return LaunchDescription([sim_rate_arg, explorer_node])
