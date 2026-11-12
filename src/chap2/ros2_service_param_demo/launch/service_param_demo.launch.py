#!/usr/bin/env python3
# launch 启动文件：ros2 launch ros2_service_param_demo service_param_demo.launch.py
# 节点与参数设置和模块入口 main.py 中的 generate_launch_description 保持一致。
"""同时启动 uav_mode_node 的 launch 启动文件（参数可由命令行覆盖）."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """生成启动 uav_mode_node 的启动描述（参数可由命令行覆盖）."""
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='2.0',
        description='状态发布频率（Hz），范围 (0, 100]')
    start_mode_arg = DeclareLaunchArgument(
        'start_mode', default_value='IDLE',
        description='启动模式：IDLE / FLYING（启动后不可修改）')

    mode_node = Node(
        package='ros2_service_param_demo',
        executable='uav_mode_node',
        name='uav_mode_node',
        output='screen',
        parameters=[{
            'publish_rate': ParameterValue(
                LaunchConfiguration('publish_rate'), value_type=float),
            'start_mode': ParameterValue(
                LaunchConfiguration('start_mode'), value_type=str),
        }],
    )
    return LaunchDescription([publish_rate_arg, start_mode_arg, mode_node])
