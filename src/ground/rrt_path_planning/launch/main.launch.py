from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="rrt_path_planning",
             executable="main",
             name="rrt_path_planning"),
    ])
