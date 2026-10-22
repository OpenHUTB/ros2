#!/usr/bin/env python3
# launch 启动文件：ros2 launch ros2_service_param_demo service_param_demo.launch.py
# 节点与参数设置和模块入口 main.py 中的 generate_launch_description 保持一致。
"""同时启动 uav_mode_node 的 launch 启动文件."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动 uav_mode_node 的启动描述."""
    mode_node = Node(
        package='ros2_service_param_demo',
        executable='uav_mode_node',
        name='uav_mode_node',
        output='screen',
        parameters=[{'publish_rate': 2.0, 'start_mode': 'IDLE'}],
    )
    return LaunchDescription([mode_node])
