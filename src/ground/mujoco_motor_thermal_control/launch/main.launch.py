"""从安装目录启动电机热模型仿真。"""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    executable = Path(get_package_prefix("mujoco_motor_thermal_control")) / "lib" / "mujoco_motor_thermal_control" / "motor_thermal_main"
    return LaunchDescription([
        DeclareLaunchArgument("steps", default_value="12000", description="Simulation steps at 0.01 s each"),
        DeclareLaunchArgument("linear", default_value="0.55"),
        DeclareLaunchArgument("angular", default_value="0.0"),
        DeclareLaunchArgument("ambient", default_value="25.0"),
        DeclareLaunchArgument("render", default_value="", description="Optional PNG path for the final simulated frame"),
        ExecuteProcess(
            cmd=[sys.executable, str(executable), "--steps", LaunchConfiguration("steps"), "--linear", LaunchConfiguration("linear"), "--angular", LaunchConfiguration("angular"), "--ambient", LaunchConfiguration("ambient"), "--render", LaunchConfiguration("render")],
            output="screen",
        ),
    ])
