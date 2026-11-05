import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    share = get_package_share_directory("uav_energy_budget")
    return LaunchDescription([
        ExecuteProcess(cmd=[
            sys.executable, "-m", "uav_energy_budget.main",
            "--input", os.path.join(share, "sample_data", "mission_profile.json"),
        ], output="screen")
    ])
