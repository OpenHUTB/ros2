# -*- coding: utf-8 -*-
"""启动在线协同分解节点。

    ros2 launch muscle_synergy_lab main.launch.py
    ros2 launch muscle_synergy_lab main.launch.py k:=4 window:=300

仿真侧发数据：
    ros2 topic pub /emg/activation std_msgs/msg/Float64MultiArray "{data: [0.3, 0.1, ...]}"
看结果：
    ros2 topic echo /emg/synergy_vaf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("muscle_synergy_lab")
    params = os.path.join(share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("k", default_value="3", description="协同数"),
        DeclareLaunchArgument("window", default_value="200", description="窗口采样点数"),
        Node(
            package="muscle_synergy_lab",
            executable="synergy_node",
            name="muscle_synergy_node",
            output="screen",
            parameters=[params, {
                "k": LaunchConfiguration("k"),
                "window": LaunchConfiguration("window"),
            }],
        ),
    ])
