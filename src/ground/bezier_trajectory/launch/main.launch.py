# -*- coding: utf-8 -*-
"""ROS 2 launch 文件：启动 Bezier 曲线可视化节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bezier_trajectory',
            executable='main',
            name='bezier_visualizer',
            output='screen',
        ),
    ])
