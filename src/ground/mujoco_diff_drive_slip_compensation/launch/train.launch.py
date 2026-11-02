"""Launch the local CPU slip-training command against collected episodes."""

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Expose dataset and artifact paths as launch arguments."""
    return LaunchDescription([
        DeclareLaunchArgument('data_directory', default_value='data/raw'),
        DeclareLaunchArgument('output_directory', default_value='data/models'),
        ExecuteProcess(
            cmd=[
                sys.executable,
                '-m',
                'mujoco_diff_drive_slip_compensation.slip_model',
                '--data', LaunchConfiguration('data_directory'),
                '--output', LaunchConfiguration('output_directory'),
            ],
            output='screen',
        ),
    ])
