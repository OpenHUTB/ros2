"""Start the ROS 2 visual landing controller."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [Node(package="mujoco_uav_moving_pad_landing", executable="landing_controller", output="screen")]
    )
