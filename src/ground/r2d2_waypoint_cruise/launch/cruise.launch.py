"""R2D2 小车航点巡航仿真启动文件

    ros2 launch r2d2_waypoint_cruise cruise.launch.py

本模块为独立 PyBullet 仿真脚本，不依赖 ROS 节点。
launch 文件仅作为统一启动入口调用 main.py。
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "run", "r2d2_waypoint_cruise", "r2d2_waypoint_cruise"],
            output="screen",
        ),
    ])
