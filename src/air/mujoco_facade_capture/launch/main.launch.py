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
        DeclareLaunchArgument("controller", default_value="proportional", description="控制器类型"),
        DeclareLaunchArgument("model", default_value=str(root / "models/facade_controller.json"), description="神经网络权重文件"),
        ExecuteProcess(cmd=[sys.executable, str(root / "main.py"), "--output", LaunchConfiguration("output"),
                            "--style", LaunchConfiguration("style"), "--controller",
                            LaunchConfiguration("controller"), "--model", LaunchConfiguration("model")], output="screen"),
    ])
