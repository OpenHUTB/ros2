# -*- coding: utf-8 -*-
"""ROS 2 launch 文件：启动 APF 路径规划可视化节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apf_path_planning',
            executable='main',
            name='apf_visualizer',
            output='screen',
        ),
    ])