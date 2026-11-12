import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("ros2_occupancy_mapping"),
        "config", "mapping_params.yaml")
    return LaunchDescription([
        Node(
            package="ros2_occupancy_mapping",
            executable="main_mapping",
            name="occupancy_mapping",
            output="screen",
            parameters=[config],
        ),
    ])
