#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业五 启动文件 —— CARLA 整合+性能评价（ROS2 Humble 语法）

用法：
    source /opt/ros/humble/setup.bash
    # 性能评价（target 为空默认跑 benchmark）
    ros2 launch carla_assignment/05_reports/launch/main.launch.py
    # 运行某一子作业
    ros2 launch carla_assignment/05_reports/launch/main.launch.py target:=navigation
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(_ROOT, "05_reports", "main.py")


def _launch_setup(context, *args, **kwargs):
    target = context.launch_configurations.get("target", "")
    cmd = ["python3", SCRIPT]
    if target:
        cmd += ["--target", target]
    else:
        cmd += ["--benchmark"]
    return [ExecuteProcess(cmd=cmd, name="carla_reports_node", output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target", default_value="",
                              choices=["", "control", "perception",
                                       "navigation", "end_to_end"]),
        OpaqueFunction(function=_launch_setup),
    ])
