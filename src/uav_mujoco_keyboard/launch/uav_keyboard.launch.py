from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', 'main.py'],
            cwd=pkg_dir,
            output='screen'
        )
    ])
