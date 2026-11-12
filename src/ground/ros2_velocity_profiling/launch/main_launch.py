import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("ros2_velocity_profiling"),
        "config", "profile_params.yaml")
    return LaunchDescription([
        Node(
            package="ros2_velocity_profiling",
            executable="main_profile",
            name="velocity_profiling",
            output="screen",
            parameters=[config],
        ),
    ])
