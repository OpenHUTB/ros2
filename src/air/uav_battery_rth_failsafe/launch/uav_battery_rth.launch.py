import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    main_py = os.path.join(script_dir, "..", "main.py")
    return LaunchDescription([
        ExecuteProcess(cmd=["python3", main_py], output="screen")
    ])
