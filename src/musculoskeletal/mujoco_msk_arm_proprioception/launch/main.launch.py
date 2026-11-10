"""Launch the arm simulation and proprioceptive sensing node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_name = 'mujoco_msk_arm_proprioception'
    arm_package = 'mujoco_musculoskeletal_arm_reach'
    arm_share = Path(get_package_share_directory(arm_package))
    share = Path(get_package_share_directory(package_name))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(arm_share / 'launch' / 'main.launch.py')
            )
        ),
        Node(
            package=package_name,
            executable='proprioception_node',
            name=package_name,
            output='screen',
            parameters=[str(share / 'config' / 'simulation.yaml')],
        ),
    ])
