import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_share = get_package_share_directory("airlib_message_regression")
    command = [
        sys.executable,
        "-m",
        "airlib_message_regression.main",
        "--input",
        os.path.join(package_share, "sample_data", "sample_messages.json"),
    ]
    return LaunchDescription([ExecuteProcess(cmd=command, output="screen")])
