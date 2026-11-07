from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="bug2_wall_following",
             executable="bug2_wall_following",
             name="bug2_wall_following"),
    ])
