from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="privacy_mask",
            executable="privacy_mask_node",
            output="screen"
        )
    ])
