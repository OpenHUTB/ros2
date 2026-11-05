"""Launch task 2: sensors, perception MLP, and neural trajectory control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start simulation, perception and trajectory control (no keyboard)."""
    package_share = get_package_share_directory('mujoco_auto_parking')
    default_config = os.path.join(package_share, 'config', 'simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='仿真参数文件 / simulation parameter file',
        ),
        DeclareLaunchArgument(
            'v_ref',
            default_value='0.45',
            description='参考速度 / reference speed (m/s)',
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
            executable='perception_node',
            name='parking_perception_node',
            output='screen',
            parameters=[{
                'weights_file': 'models/perception.npz',
            }],
        ),
        Node(
            package='mujoco_auto_parking',
            executable='trajectory_control_node',
            name='trajectory_control_node',
            output='screen',
            parameters=[{
                'weights_file': 'models/trajectory.npz',
                'trajectory': 's_curve',
                'v_ref': LaunchConfiguration('v_ref'),
            }],
        ),
    ])
