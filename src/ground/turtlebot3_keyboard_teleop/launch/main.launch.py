import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("turtlebot3_keyboard_teleop"),
        "config", "params.yaml",
    )
    return LaunchDescription([
        Node(
            package="turtlebot3_keyboard_teleop",
            executable="teleop",
            name="turtlebot3_keyboard_teleop",
            parameters=[params_file],
            output="screen",
        ),
    ])
