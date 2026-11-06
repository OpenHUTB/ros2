"""Launch task 3: mapping, Neural A* planning and neural navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start simulation and the mapping/planning/control navigation node."""
    package_share = get_package_share_directory('mujoco_auto_parking')
    default_config = os.path.join(package_share, 'config', 'simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='仿真参数文件 / simulation parameter file',
        ),
        DeclareLaunchArgument(
            'goal',
            default_value='0.25,1.2',
            description='导航目标 x,y',
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
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'planner_weights': 'models/planner.npz',
                'controller_weights': 'models/trajectory.npz',
                'goal': LaunchConfiguration('goal'),
            }],
        ),
    ])
