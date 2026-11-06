from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_slope_car")
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", os.path.join(pkg_share, "..", "..", "lib", "mujoco_slope_car", "main.py")],
            output="screen"
        )
    ])
