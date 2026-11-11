import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("ros2_dynamic_obstacle_ttc"),
        "config", "ttc_params.yaml")
    return LaunchDescription([
        Node(
            package="ros2_dynamic_obstacle_ttc",
            executable="main_ttc",
            name="dynamic_obstacle_ttc",
            output="screen",
            parameters=[config],
        ),
    ])
