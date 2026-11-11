"""Launch the standalone thermal scenario evaluator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix


def generate_launch_description() -> LaunchDescription:
    executable = Path(get_package_prefix("motor_thermal_scenario_evaluator")) / "lib" / "motor_thermal_scenario_evaluator" / "motor_thermal_evaluate"
    return LaunchDescription([
        DeclareLaunchArgument("steps", default_value="6000"),
        DeclareLaunchArgument("output", default_value="output/thermal_evaluation"),
        ExecuteProcess(
            cmd=[sys.executable, str(executable), "--steps", LaunchConfiguration("steps"), "--output", LaunchConfiguration("output")],
            output="screen",
        ),
    ])
