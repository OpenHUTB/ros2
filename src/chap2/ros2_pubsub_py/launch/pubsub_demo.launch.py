#!/usr/bin/env python3
# launch 启动文件：ros2 launch ros2_pubsub_py pubsub_demo.launch.py
# 节点列表与模块入口 main.py 中的 generate_launch_description 保持一致。
"""同时启动 talker 与 listener 的 launch 启动文件."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成同时启动 talker 与 listener 的启动描述."""
    talker_node = Node(
        package='ros2_pubsub_py',
        executable='talker',
        name='talker',
        output='screen',
    )
    listener_node = Node(
        package='ros2_pubsub_py',
        executable='listener',
        name='listener',
        output='screen',
    )
    return LaunchDescription([talker_node, listener_node])
