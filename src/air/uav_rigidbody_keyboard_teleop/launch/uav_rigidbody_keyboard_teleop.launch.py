#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动文件：启动刚体动力学遥控仿真节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_rate_arg = DeclareLaunchArgument(
        "sim_rate", default_value="250.0", description="仿真控制频率 Hz")

    teleop_node = Node(
        package="uav_rigidbody_keyboard_teleop",
        executable="teleop_node",
        name="uav_rigidbody_keyboard_teleop",
        output="screen",
        parameters=[{"sim_rate": LaunchConfiguration("sim_rate")}],
    )

    return LaunchDescription([sim_rate_arg, teleop_node])
