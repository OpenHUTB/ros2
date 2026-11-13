from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="line_following_pid",
             executable="line_following_pid",
             name="line_following_pid"),
    ])
