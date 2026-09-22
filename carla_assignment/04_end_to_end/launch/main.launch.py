#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务4 启动文件 —— CARLA 端到端 CNN（ROS2 Humble 语法）

用法：
    source /opt/ros/humble/setup.bash
    # 采集 / 训练 / 测试 三选一
    ros2 launch carla_assignment/04_end_to_end/launch/main.launch.py mode:=collect
    ros2 launch carla_assignment/04_end_to_end/launch/main.launch.py mode:=train
    ros2 launch carla_assignment/04_end_to_end/launch/main.launch.py mode:=test
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(_ROOT, "04_end_to_end", "main.py")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="2000"),
        DeclareLaunchArgument("town", default_value="Town05"),
        DeclareLaunchArgument("mode", default_value="test",
                              choices=["collect", "train", "test"]),
        DeclareLaunchArgument("frames", default_value="200"),
        DeclareLaunchArgument("epochs", default_value="30"),
        DeclareLaunchArgument("out_dir", default_value="dataset"),
        DeclareLaunchArgument("data_dir", default_value="dataset"),
        DeclareLaunchArgument("model_path", default_value="models/cnn.h5"),
        DeclareLaunchArgument("backend", default_value="tf",
                              choices=["tf", "numpy"]),
        DeclareLaunchArgument("sim_time", default_value="10.0"),
        ExecuteProcess(
            cmd=["python3", SCRIPT,
                 "--host", LaunchConfiguration("host"),
                 "--port", LaunchConfiguration("port"),
                 "--town", LaunchConfiguration("town"),
                 "--mode", LaunchConfiguration("mode"),
                 "--frames", LaunchConfiguration("frames"),
                 "--epochs", LaunchConfiguration("epochs"),
                 "--out_dir", LaunchConfiguration("out_dir"),
                 "--data_dir", LaunchConfiguration("data_dir"),
                 "--model_path", LaunchConfiguration("model_path"),
                 "--backend", LaunchConfiguration("backend"),
                 "--sim_time", LaunchConfiguration("sim_time")],
            name="carla_end_to_end_node",
            output="screen",
        ),
    ])
