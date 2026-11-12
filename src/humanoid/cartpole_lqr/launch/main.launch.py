#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Python launch：启动 cartpole_lqr 的演示节点。

    ros2 launch cartpole_lqr main.launch.py
    ros2 launch cartpole_lqr main.launch.py use_lqr:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_lqr", default_value="true",
            description="是否启用 LQR（false 为自由倒下对照）",
        ),
        Node(
            package="cartpole_lqr",
            executable="ros_node.py",
            name="cartpole_lqr_demo",
            output="screen",
            parameters=[
                {
                "use_lqr": ParameterValue(LaunchConfiguration("use_lqr"), value_type=bool)
                }
            ],
        ),
    ])
