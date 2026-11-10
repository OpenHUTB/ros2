"""Launch task 4: image-to-control end-to-end driving."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start simulation and the end-to-end vision controller."""
    package_share = get_package_share_directory('mujoco_auto_parking')
    default_config = os.path.join(package_share, 'config', 'simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='仿真参数文件 / simulation parameter file',
        ),
        Node(
            package='mujoco_auto_parking',
            executable='simulation_node',
            name='mujoco_auto_parking_simulation',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        Node(
            package='mujoco_auto_parking',
            executable='end_to_end_node',
            name='end_to_end_node',
            output='screen',
            parameters=[{
                'weights_file': 'models/e2e_small_cnn_bc.npz',
                'architecture': 'small_cnn',
                'nominal_speed': 0.45,
            }],
        ),
    ])
