import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_gimbal_reacquisition"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="0.0"),
            DeclareLaunchArgument("rate", default_value="20.0"),
            Node(
                package="uav_gimbal_reacquisition",
                executable="gimbal_target_sim",
                name="uav_gimbal_target_sim",
                output="screen",
                parameters=[
                    config,
                    {"duration_s": ParameterValue(LaunchConfiguration("duration"), value_type=float)},
                ],
            ),
            Node(
                package="uav_gimbal_reacquisition",
                executable="gimbal_tracker",
                name="uav_gimbal_tracker",
                output="screen",
                parameters=[
                    config,
                    {"rate_hz": ParameterValue(LaunchConfiguration("rate"), value_type=float)},
                ],
            ),
        ]
    )
