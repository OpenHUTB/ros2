"""Launch MuJoCo with online slip compensation in the command path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetRemap,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Run the simulator on compensated commands and expose raw input."""
    package_share = get_package_share_directory(
        'mujoco_diff_drive_slip_compensation'
    )
    simulator_launch = os.path.join(
        package_share, 'launch', 'main.launch.py'
    )
    config_path = os.path.join(package_share, 'config', 'compensation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_file', default_value='data/models/slip-v1/slip_model.json'
        ),
        GroupAction([
            SetRemap(src='cmd_vel', dst='cmd_vel/compensated'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_launch)
            ),
        ]),
        Node(
            package='mujoco_diff_drive_slip_compensation',
            executable='online_slip_compensation',
            name='online_slip_compensation',
            output='screen',
            parameters=[
                config_path,
                {'model_file': LaunchConfiguration('model_file')},
            ],
        ),
    ])
