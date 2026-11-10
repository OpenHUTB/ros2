from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Explicit paths make the source data/model choices visible, avoiding hidden user directories.
    return LaunchDescription([
        DeclareLaunchArgument('model', description='Absolute model directory containing model.pt and config.json'),
        DeclareLaunchArgument('csv', description='Absolute observations.csv replay path'),
        DeclareLaunchArgument('plot', default_value='true'),
        Node(package='telemetry_diagnostics', executable='telemetry_diagnostics',
             parameters=[{'model': LaunchConfiguration('model')}]),
        Node(package='telemetry_diagnostics', executable='telemetry_replay',
             parameters=[{'csv': LaunchConfiguration('csv'), 'startup_delay': 3.}]),
        Node(package='telemetry_diagnostics', executable='telemetry_dashboard',
             condition=IfCondition(LaunchConfiguration('plot'))),
    ])
