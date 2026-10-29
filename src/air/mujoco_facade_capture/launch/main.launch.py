"""通过 ROS 2 launch 启动完整 MuJoCo 扫描任务。"""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="/tmp/facade_scan", description="扫描结果目录"),
        DeclareLaunchArgument("style", default_value="grid", description="立面类型"),
        ExecuteProcess(cmd=[sys.executable, str(root / "main.py"), "--output", LaunchConfiguration("output"),
                            "--style", LaunchConfiguration("style")], output="screen"),
    ])
