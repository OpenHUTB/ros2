"""通过 ROS 2 launch 启动 DAVE 水下演示（实际运行依赖 ROS 1 Noetic + Gazebo 工作区）。

用法：
  ros2 launch dave_underwater_sim main.launch.py            # demo 场景（默认）
  ros2 launch dave_underwater_sim main.launch.py mode:=valve
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="demo",
                description="DAVE underwater demo scene: demo | valve | plug",
            ),
            Node(
                package="dave_underwater_sim",
                executable="dave_underwater_sim",
                name="dave_underwater_sim",
                output="screen",
                arguments=[mode],
            ),
        ]
    )
