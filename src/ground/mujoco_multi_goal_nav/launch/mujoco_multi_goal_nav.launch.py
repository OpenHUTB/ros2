from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', os.path.join(os.path.dirname(__file__), '../main.py')],
            output='screen'
        )
    ])
