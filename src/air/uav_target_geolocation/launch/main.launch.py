#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动目标地理定位在线判定节点。

    ros2 launch uav_target_geolocation main.launch.py
    ros2 launch uav_target_geolocation main.launch.py params_file:=/path/to/x.yaml

参数只在一处定义：``config/uav_target_geolocation.yaml``。这里不再抄第二份
（抄一遍的结果就是改了一边忘了另一边），需要换参数时用 ``params_file``。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_params_file() -> str:
    """装好的包里取 share/config，源码目录直接 launch 时退回仓库内路径。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("uav_target_geolocation")
        cand = os.path.join(share, "config", "uav_target_geolocation.yaml")
        if os.path.isfile(cand):
            return cand
    except Exception:                        # pragma: no cover - 无 ROS 环境
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "config",
                                         "uav_target_geolocation.yaml"))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=_default_params_file(),
            description="节点参数文件（默认包内 config/uav_target_geolocation.yaml）",
        ),
        Node(
            package="uav_target_geolocation",
            executable="geo_node",
            name="geo_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
