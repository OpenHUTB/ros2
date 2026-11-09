#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 launch：规划一条 Dubins 路径并发布成 nav_msgs/Path。

    ros2 launch ackermann_dubins_path ackermann_dubins_path.launch.py
    ros2 launch ackermann_dubins_path ackermann_dubins_path.launch.py goal_yaw:=1.5708 radius:=2.0
    rviz2   # 添加 Path 显示，Topic 选 /dubins/path
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_x", default_value="0.0", description="起点 x (m)"),
        DeclareLaunchArgument("start_y", default_value="0.0", description="起点 y (m)"),
        DeclareLaunchArgument("start_yaw", default_value="0.0", description="起点航向 (rad)"),
        DeclareLaunchArgument("goal_x", default_value="10.0", description="终点 x (m)"),
        DeclareLaunchArgument("goal_y", default_value="0.0", description="终点 y (m)"),
        DeclareLaunchArgument("goal_yaw", default_value="3.14159265",
                              description="终点航向 (rad)"),
        DeclareLaunchArgument("radius", default_value="1.0",
                              description="最小转弯半径 (m)"),
        Node(
            package="ackermann_dubins_path",
            executable="ros_node.py",
            name="dubins_path",
            output="screen",
            parameters=[{
                "start_x": ParameterValue(LaunchConfiguration("start_x"), value_type=float),
                "start_y": ParameterValue(LaunchConfiguration("start_y"), value_type=float),
                "start_yaw": ParameterValue(LaunchConfiguration("start_yaw"), value_type=float),
                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                "goal_yaw": ParameterValue(LaunchConfiguration("goal_yaw"), value_type=float),
                "radius": ParameterValue(LaunchConfiguration("radius"), value_type=float),
            }],
        ),
    ])
