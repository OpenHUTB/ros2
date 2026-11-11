import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("ros2_costmap_inflation"),
        "config", "inflation_params.yaml")
    return LaunchDescription([
        Node(
            package="ros2_costmap_inflation",
            executable="main_inflation",
            name="costmap_inflation",
            output="screen",
            parameters=[config],
        ),
    ])
