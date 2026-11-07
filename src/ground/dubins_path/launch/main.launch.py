# -*- coding: utf-8 -*-
"""ROS 2 launch 文件：启动 Dubins 曲线可视化节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dubins_path',
            executable='main',
            name='dubins_visualizer',
            output='screen',
        ),
    ])
