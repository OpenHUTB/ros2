#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 launch：起一个 MPC 跟踪节点。

    ros2 launch ackermann_mpc_tracking ackermann_mpc_tracking.launch.py
    ros2 topic echo /cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("v_cruise", default_value="5.0",
                              description="纵向速度 (m/s)"),
        DeclareLaunchArgument("horizon", default_value="15",
                              description="预测步数"),
        DeclareLaunchArgument("rate_max", default_value="1.6",
                              description="转向速率上限 (rad/s)"),
        Node(
            package="ackermann_mpc_tracking",
            executable="ros_node.py",
            name="mpc_tracking",
            output="screen",
            parameters=[{
                "v_cruise": ParameterValue(LaunchConfiguration("v_cruise"), value_type=float),
                "horizon": ParameterValue(LaunchConfiguration("horizon"), value_type=int),
                "rate_max": ParameterValue(LaunchConfiguration("rate_max"), value_type=float),
            }],
        ),
    ])
