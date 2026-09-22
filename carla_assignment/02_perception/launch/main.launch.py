#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2 启动文件 —— CARLA 感知 + 轨迹跟踪（ROS2 Humble 语法）

用法：
    source /opt/ros/humble/setup.bash
    ros2 launch carla_assignment/02_perception/launch/main.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(_ROOT, "02_perception", "main.py")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="2000"),
        DeclareLaunchArgument("town", default_value="Town05"),
        DeclareLaunchArgument("mode", default_value="run",
                              choices=["train", "run"]),
        DeclareLaunchArgument("model", default_value="models/nn_percept.json"),
        DeclareLaunchArgument("waypoints", default_value="40,-8 40,12 25,20"),
        DeclareLaunchArgument("sim_time", default_value="25.0"),
        ExecuteProcess(
            cmd=["python3", SCRIPT,
                 "--host", LaunchConfiguration("host"),
                 "--port", LaunchConfiguration("port"),
                 "--town", LaunchConfiguration("town"),
                 "--mode", LaunchConfiguration("mode"),
                 "--model", LaunchConfiguration("model"),
                 "--waypoints", LaunchConfiguration("waypoints"),
                 "--sim_time", LaunchConfiguration("sim_time")],
            name="carla_perception_node",
            output="screen",
        ),
    ])
