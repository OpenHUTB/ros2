"""启动独立物理仿真。 Launch the standalone physics simulation."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    module = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value=str(module / "results")),
        DeclareLaunchArgument("controller", default_value="range", choices=["reference", "range", "mlp"]),
        DeclareLaunchArgument("model", default_value=str(module / "models" / "mlp_controller.json")),
        ExecuteProcess(cmd=[sys.executable, str(module / "main.py"), "--headless",
                            "--output", LaunchConfiguration("output"),
                            "--controller", LaunchConfiguration("controller"),
                            "--model", LaunchConfiguration("model")], output="screen"),
    ])
