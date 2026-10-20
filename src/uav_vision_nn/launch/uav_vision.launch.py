from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    script = os.path.expanduser("~/ros2/src/uav_vision_nn/main.py")
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script],
            output='screen'
        )
    ])
