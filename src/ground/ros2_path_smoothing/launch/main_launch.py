import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("ros2_path_smoothing"),
        "config", "smoothing_params.yaml")
    return LaunchDescription([
        Node(
            package="ros2_path_smoothing",
            executable="main_smooth",
            name="path_smoothing",
            output="screen",
            parameters=[config],
        ),
    ])
