"""Launch held-out slip-model evaluation from episode manifests."""

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Expose dataset, model and result paths as launch arguments."""
    return LaunchDescription([
        DeclareLaunchArgument('data_directory', default_value='data/sample'),
        DeclareLaunchArgument(
            'model_file', default_value='data/models/slip-v1/slip_model.json'
        ),
        DeclareLaunchArgument(
            'result_file', default_value='data/results/slip-evaluation.json'
        ),
        ExecuteProcess(
            cmd=[
                sys.executable,
                '-m',
                'mujoco_diff_drive_slip_compensation.evaluate_slip_model',
                '--data', LaunchConfiguration('data_directory'),
                '--model', LaunchConfiguration('model_file'),
                '--output', LaunchConfiguration('result_file'),
            ],
            output='screen',
        ),
    ])
