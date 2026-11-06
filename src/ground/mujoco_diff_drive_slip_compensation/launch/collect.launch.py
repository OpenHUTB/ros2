"""Launch simulation, deterministic commands, and episode-aware collection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the complete multi-friction collection workflow."""
    package_share = get_package_share_directory(
        'mujoco_diff_drive_slip_compensation'
    )
    simulation_config = os.path.join(package_share, 'config', 'simulation.yaml')
    collection_config = os.path.join(package_share, 'config', 'collection.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_directory',
            default_value='data/raw',
            description='数据输出目录 / dataset output directory',
        ),
        Node(
            package='mujoco_diff_drive_slip_compensation',
            executable='main',
            name='mujoco_diff_drive_simulation',
            output='screen',
            parameters=[simulation_config],
        ),
        Node(
            package='mujoco_diff_drive_slip_compensation',
            executable='command_profile',
            name='slip_collection_command_profile',
            output='screen',
            parameters=[collection_config],
        ),
        Node(
            package='mujoco_diff_drive_slip_compensation',
            executable='collect_dataset',
            name='mujoco_slip_data_collector',
            output='screen',
            parameters=[
                collection_config,
                {'output_directory': LaunchConfiguration('output_directory')},
            ],
        ),
    ])
