"""Launch the survey CLI through ROS 2, previewing by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = (
    ("length", "60"),
    ("width", "40"),
    ("lane_spacing", "8"),
    ("altitude", "20"),
    ("speed", "5"),
    ("ip", "127.0.0.1"),
    ("port", "41451"),
    ("vehicle", ""),
    ("execute", "false"),
)


def _arguments():
    arguments = []
    for name in ARGUMENTS[:-1]:
        arguments.extend((f"--{name[0].replace('_', '-')}", LaunchConfiguration(name[0])))
    return arguments


def generate_launch_description():
    """Return a reproducible preview/execute launch description."""

    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in ARGUMENTS
    ]
    arguments = _arguments()
    return LaunchDescription(
        declarations
        + [
            Node(
                package="drone_lawnmower_survey",
                executable="survey",
                arguments=arguments,
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("execute")),
            ),
            Node(
                package="drone_lawnmower_survey",
                executable="survey",
                arguments=arguments + ["--execute"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("execute")),
            ),
        ]
    )
