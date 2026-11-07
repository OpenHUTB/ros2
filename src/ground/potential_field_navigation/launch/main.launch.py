from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="potential_field_navigation",
             executable="potential_field_navigation",
             name="potential_field_navigation"),
    ])
