"""Launch the MuJoCo auto-parking simulation and keyboard teleop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build a parameterized ROS 2 launch description."""
    package_share = get_package_share_directory('mujoco_auto_parking')
    default_config = os.path.join(package_share, 'config', 'simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='参数文件 / parameter file',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='true',
            description='是否启动键盘节点 / start keyboard teleop',
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
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('teleop')),
            parameters=[{'linear_speed': 0.35, 'angular_speed': 0.9}],
        ),
    ])

