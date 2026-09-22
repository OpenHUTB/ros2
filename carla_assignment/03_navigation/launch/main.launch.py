#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务3 启动文件 —— CARLA 建图 + 导航（ROS2 Humble 语法）

用法：
    source /opt/ros/humble/setup.bash
    ros2 launch carla_assignment/03_navigation/launch/main.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(_ROOT, "03_navigation", "main.py")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="2000"),
        DeclareLaunchArgument("town", default_value="Town05"),
        DeclareLaunchArgument("mode", default_value="run",
                              choices=["train", "run"]),
        DeclareLaunchArgument("model", default_value="models/nn_plan.json"),
        DeclareLaunchArgument("goal", default_value="20,8"),
        DeclareLaunchArgument("sim_time", default_value="25.0"),
        ExecuteProcess(
            cmd=["python3", SCRIPT,
                 "--host", LaunchConfiguration("host"),
                 "--port", LaunchConfiguration("port"),
                 "--town", LaunchConfiguration("town"),
                 "--mode", LaunchConfiguration("mode"),
                 "--model", LaunchConfiguration("model"),
                 "--goal", LaunchConfiguration("goal"),
                 "--sim_time", LaunchConfiguration("sim_time")],
            name="carla_mapping_nav_node",
            output="screen",
        ),
    ])
