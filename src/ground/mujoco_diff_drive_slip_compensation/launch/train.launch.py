"""Launch the local CPU slip-training command against collected episodes."""

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Expose dataset, artifact, and training hyperparameters as launch arguments."""

    return LaunchDescription([
        DeclareLaunchArgument('data_directory', default_value='data/raw'),
        DeclareLaunchArgument('output_directory', default_value='data/models/slip-v1'),
        DeclareLaunchArgument('window_size', default_value='12'),
        DeclareLaunchArgument('hidden_size', default_value='24'),
        DeclareLaunchArgument('epochs', default_value='300'),
        DeclareLaunchArgument('seed', default_value='25012'),
        ExecuteProcess(
            cmd=[
                sys.executable,
                '-m',
                'mujoco_diff_drive_slip_compensation.slip_model',
                '--data', LaunchConfiguration('data_directory'),
                '--output', LaunchConfiguration('output_directory'),
                '--window-size', LaunchConfiguration('window_size'),
                '--hidden-size', LaunchConfiguration('hidden_size'),
                '--epochs', LaunchConfiguration('epochs'),
                '--seed', LaunchConfiguration('seed'),
            ],
            output='screen',
        ),
    ])
