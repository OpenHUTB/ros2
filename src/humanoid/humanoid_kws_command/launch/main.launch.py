# -*- coding: utf-8 -*-
"""ROS 2 launch：离线 WAV 关键词识别 -> 指令话题。

    ros2 launch humanoid_kws_command main.launch.py
    ros2 launch humanoid_kws_command main.launch.py wav_path:=/path/to/stop.wav model_path:=/path/to/kws_mlp.npz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("humanoid_kws_command")
    args = [
        DeclareLaunchArgument("wav_path", default_value="", description="待识别的 WAV"),
        DeclareLaunchArgument("model_path", default_value="", description="MLP 权重 .npz"),
        DeclareLaunchArgument("repeat_hz", default_value="0.5"),
    ]
    node = Node(
        package="humanoid_kws_command",
        executable="kws_command_node",
        name="kws_command_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg, "config", "params.yaml"]),
            {
                "wav_path": LaunchConfiguration("wav_path"),
                "model_path": LaunchConfiguration("model_path"),
                "repeat_hz": LaunchConfiguration("repeat_hz"),
            },
        ],
    )
    return LaunchDescription(args + [node])