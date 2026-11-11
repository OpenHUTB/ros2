"""Run the standalone temperature forecast evaluator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("csv"),
        DeclareLaunchArgument("model", default_value="output/temperature_model.json"),
        DeclareLaunchArgument("results", default_value="output/forecast_results.json"),
        Node(
            package="mujoco_motor_temperature_predictor",
            executable="motor_temperature_forecast",
            arguments=["--csv", LaunchConfiguration("csv"), "--model", LaunchConfiguration("model"), "--results", LaunchConfiguration("results")],
            output="screen",
        ),
    ])
