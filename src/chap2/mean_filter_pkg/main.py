#!/usr/bin/env python3
"""
mean_filter_pkg 模块入口：以 launch 方式启动均值滤波节点.

以下三种启动方式等价（需先 colcon build 并 source install/setup.bash）：
    python3 main.py
    ros2 launch mean_filter_pkg mean_filter.launch.py
    ros2 run mean_filter_pkg mean_filter
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """包含本模块的 launch 文件并返回启动描述."""
    launch_file = os.path.join(
        get_package_share_directory('mean_filter_pkg'),
        'launch', 'mean_filter.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file)),
    ])


def main():
    """运行 launch 服务并返回进程退出码."""
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == '__main__':
    raise SystemExit(main())
