"""启动 ROS 2 测距节点。 Launch the ROS 2 ranging node."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    module = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="mlp", choices=["reference", "range", "mlp"]),
        DeclareLaunchArgument("model", default_value=str(module / "models" / "mlp_controller.json")),
        DeclareLaunchArgument(
            "python3",
            default_value=sys.executable,
            description="Python executable containing both ROS 2 and MuJoCo",
        ),
        ExecuteProcess(cmd=[LaunchConfiguration("python3"), str(module / "ros_node.py"),
                            "--controller", LaunchConfiguration("controller"),
                            "--model", LaunchConfiguration("model")], output="screen"),
    ])
