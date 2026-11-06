"""Launch the six-muscle two-joint MuJoCo arm."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_name = 'mujoco_musculoskeletal_arm_reach'
    share = Path(get_package_share_directory(package_name))
    return LaunchDescription([
        Node(
            package=package_name,
            executable='arm_sim_node',
            name=package_name,
            output='screen',
            parameters=[str(share / 'config' / 'simulation.yaml')],
        ),
    ])
