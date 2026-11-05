"""Start the ROS 2 visual landing controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_mode",
                default_value="features",
                description="Controller input: normalized features or RGB camera images",
            ),
            Node(
                package="mujoco_uav_moving_pad_landing",
                executable="landing_controller",
                output="screen",
                parameters=[{"input_mode": LaunchConfiguration("input_mode")}],
            ),
        ]
    )
