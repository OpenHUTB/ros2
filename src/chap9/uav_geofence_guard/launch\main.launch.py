from launch import LaunchDescription
from launch.actions import ExecuteProcess
from pathlib import Path
import sys


def generate_launch_description():
    entry = Path(__file__).resolve().parents[1] / 'main.py'
    return LaunchDescription([ExecuteProcess(cmd=[sys.executable, str(entry)], output='screen')])
