#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Python launch：启动 quad_eso_hover 的演示节点。

    ros2 launch quad_eso_hover main.launch.py
    ros2 launch quad_eso_hover main.launch.py w0:=15.0 ref:=1.0 rate:=200.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "w0", default_value="15.0",
            description="ESO 观测器带宽 (rad/s)",
        ),
        DeclareLaunchArgument(
            "ref", default_value="1.0",
            description="目标高度 (m)",
        ),
        DeclareLaunchArgument(
            "rate", default_value="200.0",
            description="控制频率 (Hz)",
        ),
        Node(
            package="quad_eso_hover",
            executable="ros_node.py",
            name="quad_eso_hover_demo",
            output="screen",
            parameters=[
                {
                "w0": ParameterValue(LaunchConfiguration("w0"), value_type=float),
                "ref": ParameterValue(LaunchConfiguration("ref"), value_type=float),
                "rate": ParameterValue(LaunchConfiguration("rate"), value_type=float),
                }
            ],
        ),
    ])
