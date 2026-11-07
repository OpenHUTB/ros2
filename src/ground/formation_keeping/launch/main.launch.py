from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="formation_keeping",
             executable="formation_keeping",
             name="formation_keeping"),
    ])
