"""Run the independent CSV sensor recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("input"),
        DeclareLaunchArgument("output", default_value="output/motor_sensors.csv"),
        Node(
            package="mujoco_motor_sensor_recorder",
            executable="motor_sensor_record",
            arguments=["--input", LaunchConfiguration("input"), "--output", LaunchConfiguration("output")],
            output="screen",
        ),
    ])
