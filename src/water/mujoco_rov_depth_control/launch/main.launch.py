"""ROS 2 launch wrapper for mujoco_rov_depth_control.

用法：
    ros2 launch mujoco_rov_depth_control main.launch.py

本模块通过 CARLA / AirSim / MuJoCo 的 Python API 直连仿真器（不启动 carla_ros_bridge），
launch 文件只负责参数声明与进程包装；可执行文件由 setup.py 的 console_scripts 注册。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_rov_depth_control',
            executable='mujoco_rov_depth_control',   # console_scripts 里的入口，指向 mujoco_rov_depth_control.app:main
            name='mujoco_rov_depth_control',
            output='screen',
            arguments=[],
        ),
    ])
