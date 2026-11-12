"""占位 launch：以子进程启动 pygame 演示，本包不创建 rclpy 节点。"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "run", "formation_keeping", "formation_keeping"],
            output="screen",
        ),
    ])
