#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Python launch：启动 idm_car_following 的演示节点。

    ros2 launch idm_car_following main.launch.py
    ros2 launch idm_car_following main.launch.py rate:=50.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rate", default_value="50.0",
            description="控制频率 (Hz)",
        ),
        Node(
            package="idm_car_following",
            executable="ros_node.py",
            name="idm_car_following_demo",
            output="screen",
            parameters=[
                {
                "rate": ParameterValue(LaunchConfiguration("rate"), value_type=float)
                }
            ],
        ),
    ])
