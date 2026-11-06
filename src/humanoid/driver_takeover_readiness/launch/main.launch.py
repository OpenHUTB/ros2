from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="driver_takeover_readiness",
            executable="takeover_readiness_node",
            name="driver_takeover_readiness",
            output="screen",
        )
    ])
