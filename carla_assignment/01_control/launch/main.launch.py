#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1 启动文件 —— 启动 CARLA 无人车键盘控制（ROS2 Humble 语法）

用法：
    source /opt/ros/humble/setup.bash
    ros2 launch carla_assignment/01_control/launch/main.launch.py

需要先启动 CARLA 服务端：./CarlaUE4.sh -carla-rpc-port=2000
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(_ROOT, "01_control", "main.py")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="2000"),
        DeclareLaunchArgument("town", default_value="Town05"),
        DeclareLaunchArgument("sim_time", default_value="0.0"),
        ExecuteProcess(
            cmd=["python3", SCRIPT,
                 "--host", LaunchConfiguration("host"),
                 "--port", LaunchConfiguration("port"),
                 "--town", LaunchConfiguration("town"),
                 "--sim_time", LaunchConfiguration("sim_time")],
            name="carla_control_node",
            output="screen",
        ),
    ])
