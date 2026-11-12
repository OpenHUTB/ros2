#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动地形跟随在线节点。

    ros2 launch uav_terrain_follow_agl main.launch.py
    ros2 launch uav_terrain_follow_agl main.launch.py agl_set_m:=20.0 strategy:=reactive_agl

参数文件用 config/uav_terrain_follow_agl.yaml（已按 ROS 2 要求写好
``<节点名> / ros__parameters`` 两层），同时把关键项作为可覆盖的 launch 参数
显式传一遍，便于快速试不同策略。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("uav_terrain_follow_agl")
    param_file = os.path.join(share, "config", "uav_terrain_follow_agl.yaml")

    args = [
        DeclareLaunchArgument("agl_set_m", default_value="15.0",
                              description="目标离地高度（米）"),
        DeclareLaunchArgument("strategy", default_value="dem_preview",
                              description="constant_msl / reactive_agl / dem_preview"),
        DeclareLaunchArgument("climb_rate_max_mps", default_value="3.0",
                              description="爬升率上限（米/秒）"),
    ]

    node = Node(
        package="uav_terrain_follow_agl",
        executable="terrain_follow_node",
        name="terrain_follow_node",
        output="screen",
        parameters=[
            param_file,
            {
                "agl_set_m": LaunchConfiguration("agl_set_m"),
                "strategy": LaunchConfiguration("strategy"),
                "climb_rate_max_mps": LaunchConfiguration("climb_rate_max_mps"),
            },
        ],
    )
    return LaunchDescription(args + [node])
