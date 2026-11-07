"""ROS 2 launch wrapper for carla_lidar_obstacle_detection.

Usage:
    ros2 launch carla_lidar_obstacle_detection main.launch.py

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