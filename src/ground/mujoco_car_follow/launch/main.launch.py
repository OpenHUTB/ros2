from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    prefix = get_package_prefix("mujoco_car_follow")
    main_py = os.path.join(prefix, "lib", "mujoco_car_follow", "main.py")
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", main_py],
            output="screen"
        )
    ])
