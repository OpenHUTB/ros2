import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    share = get_package_share_directory("fixed_wing_sitl_preflight")
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    "-m",
                    "fixed_wing_sitl_preflight.main",
                    "--input",
                    os.path.join(share, "sample_data", "fixed_wing_profile.json"),
                ],
                output="screen",
            )
        ]
    )
