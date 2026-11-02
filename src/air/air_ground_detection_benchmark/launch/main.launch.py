import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    share = get_package_share_directory("air_ground_detection_benchmark")
    return LaunchDescription([
        ExecuteProcess(cmd=[
            sys.executable, "-m", "air_ground_detection_benchmark.main",
            "--input", os.path.join(share, "sample_data", "sample_frames.json"),
        ], output="screen")
    ])
