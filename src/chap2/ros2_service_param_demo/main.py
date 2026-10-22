#!/usr/bin/env python3
# 模块入口：直接运行本文件即可启动整个模块。
# 等价的启动方式：
#     python3 main.py
#     ros2 launch ros2_service_param_demo service_param_demo.launch.py
#     ros2 run ros2_service_param_demo uav_mode_node
"""运行本文件即启动整个模块."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动 uav_mode_node 的启动描述."""
    mode_node = Node(
        package='ros2_service_param_demo',
        executable='uav_mode_node',
        name='uav_mode_node',
        output='screen',
        # 在 launch 里给参数赋初值；运行中可用 ros2 param set 修改
        parameters=[{'publish_rate': 2.0, 'start_mode': 'IDLE'}],
    )
    return LaunchDescription([mode_node])


def main():
    """以 LaunchService 运行整个模块，返回进程退出码."""
    from launch import LaunchService

    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == '__main__':
    raise SystemExit(main())
