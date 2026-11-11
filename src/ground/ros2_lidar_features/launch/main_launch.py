import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("ros2_lidar_features"),
        "config", "features_params.yaml")
    return LaunchDescription([
        Node(
            package="ros2_lidar_features",
            executable="main_features",
            name="lidar_features",
            output="screen",
            parameters=[config],
        ),
    ])
