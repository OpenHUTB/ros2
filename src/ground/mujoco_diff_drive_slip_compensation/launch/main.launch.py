"""Launch the MuJoCo differential-drive simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build a parameterized ROS 2 launch description."""
    package_share = get_package_share_directory(
        'mujoco_diff_drive_slip_compensation'
    )
    default_config = os.path.join(package_share, 'config', 'simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='参数文件 / parameter file',
        ),
        Node(
            package='mujoco_diff_drive_slip_compensation',
            executable='main',
            name='mujoco_diff_drive_simulation',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
