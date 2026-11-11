import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg = get_package_share_directory("carla_nn_roundabout_entry")
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", "-m", "carla_nn_roundabout_entry.main", "--mode", "demo"],
            output="screen",
        ),
    ])
