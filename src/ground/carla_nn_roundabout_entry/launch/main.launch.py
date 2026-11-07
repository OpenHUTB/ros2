from launch import LaunchDescription
from launch.actions import ExecuteProcess
import sys


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, "-m", "carla_nn_roundabout_entry.main", "--steps", "360"],
            output="screen",
        )
    ])
