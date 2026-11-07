"""ROS 2 launch wrapper for mujoco_rov_depth_control.

Usage:
    ros2 launch mujoco_rov_depth_control main.launch.py

The module talks to the simulator through the CARLA / AirSim / MuJoCo Python API directly,
so this launch file only wraps the process -- it does not start carla_ros_bridge.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[__CMD__],
            output='screen',
        ),
    ])