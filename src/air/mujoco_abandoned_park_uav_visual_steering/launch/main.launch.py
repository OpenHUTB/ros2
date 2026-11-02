"""Launch the visual steering ROS 2 node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="mujoco_abandoned_park_uav_visual_steering",
            executable="visual_steering_node",
            name="visual_steering_node",
            output="screen",
        ),
    ])
