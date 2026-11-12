"""Launch the independent temperature guard node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("limit_c", default_value="65.0"),
        DeclareLaunchArgument("release_c", default_value="60.0"),
        DeclareLaunchArgument("horizon_s", default_value="5.0"),
        DeclareLaunchArgument("temperature_timeout_s", default_value="0.5"),
        Node(
            package="mujoco_motor_thermal_guard",
            executable="motor_thermal_guard",
            output="screen",
            parameters=[{
                "limit_c": ParameterValue(LaunchConfiguration("limit_c"), value_type=float),
                "release_c": ParameterValue(LaunchConfiguration("release_c"), value_type=float),
                "horizon_s": ParameterValue(LaunchConfiguration("horizon_s"), value_type=float),
                "temperature_timeout_s": ParameterValue(LaunchConfiguration("temperature_timeout_s"), value_type=float),
            }],
        ),
    ])
