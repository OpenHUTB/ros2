"""Launch the standalone thermal scenario evaluator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("steps", default_value="6000"),
        DeclareLaunchArgument("output", default_value="output/thermal_evaluation"),
        Node(
            package="motor_thermal_scenario_evaluator",
            executable="motor_thermal_evaluate",
            arguments=["--steps", LaunchConfiguration("steps"), "--output", LaunchConfiguration("output")],
            output="screen",
        ),
    ])
