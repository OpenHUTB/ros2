#!/usr/bin/env python3
# 模块入口（本地源码调试用）：python3 main.py [--ros-args ...]
# 注意：本文件不会安装进 install 空间，运行前必须先 source 工作空间环境：
#     source ~/ros2_ws/install/setup.bash
# 等价的正式启动方式：
#     ros2 launch ros2_service_param_demo service_param_demo.launch.py
"""运行本文件即启动整个模块（本地源码调试入口）."""

import sys

from launch import LaunchDescription, LaunchService
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


def main():
    """以 LaunchService 运行整个模块，并把命令行参数透传给 launch."""
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == '__main__':
    raise SystemExit(main())
