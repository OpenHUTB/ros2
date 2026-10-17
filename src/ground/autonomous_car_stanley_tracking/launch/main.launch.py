#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 启动入口：Stanley 横向跟踪节点。

用法::

    ros2 launch autonomous_car_stanley_tracking main.launch.py
    ros2 launch autonomous_car_stanley_tracking main.launch.py scenario:=circle
    ros2 launch autonomous_car_stanley_tracking main.launch.py k:=2.4 controller:=stanley

启动后节点会：
1. 载入内置参考路径（或等待 /reference_path）；
2. 以 control_rate_hz 频率订阅 /odom、发布 /cmd_vel 与 /stanley/tracking_error。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("autonomous_car_stanley_tracking")
    params_file = os.path.join(share_dir, "config", "params.yaml")

    arguments = [
        DeclareLaunchArgument("controller", default_value="stanley",
                              description="横向控制器：stanley / pure_pursuit / lookahead_pid"),
        DeclareLaunchArgument("scenario", default_value="circle",
                              description="内置参考路径：straight / circle / sine / double_lane_change"),
        DeclareLaunchArgument("k", default_value="1.6", description="Stanley 横向增益"),
        DeclareLaunchArgument("k_soft", default_value="1.2", description="Stanley 软化项"),
        DeclareLaunchArgument("target_speed", default_value="5.0", description="巡航速度 m/s"),
        DeclareLaunchArgument("feedforward", default_value="true", description="是否启用曲率前馈"),
        DeclareLaunchArgument("wait_for_path", default_value="true",
                              description="是否等待 /reference_path 话题"),
        DeclareLaunchArgument("params_file", default_value=params_file,
                              description="参数文件路径"),
    ]

    node = Node(
        package="autonomous_car_stanley_tracking",
        executable="stanley_tracking_node",
        name="stanley_tracking_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "controller": LaunchConfiguration("controller"),
                "reference_scenario": LaunchConfiguration("scenario"),
                "k": LaunchConfiguration("k"),
                "k_soft": LaunchConfiguration("k_soft"),
                "target_speed": LaunchConfiguration("target_speed"),
                "feedforward": LaunchConfiguration("feedforward"),
                "wait_for_path": LaunchConfiguration("wait_for_path"),
            },
        ],
    )

    return LaunchDescription(arguments + [node])
