#!/usr/bin/env python3
# 模块入口：直接运行本文件即可启动整个模块（talker + listener）。
# 等价的启动方式：
#     python3 main.py
#     ros2 launch ros2_pubsub_py pubsub_demo.launch.py
"""运行本文件即启动整个模块（talker + listener）."""

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


def main():
    """以 LaunchService 运行整个模块，返回进程退出码."""
    from launch import LaunchService

    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == '__main__':
    raise SystemExit(main())
