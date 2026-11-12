"""Run the standalone temperature forecast evaluator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix


def generate_launch_description() -> LaunchDescription:
    executable = Path(get_package_prefix("mujoco_motor_temperature_predictor")) / "lib" / "mujoco_motor_temperature_predictor" / "motor_temperature_forecast"
    return LaunchDescription([
        DeclareLaunchArgument("csv"),
        DeclareLaunchArgument("horizon", default_value="5.0", description="预测时域，单位秒，默认 5.0"),
        DeclareLaunchArgument("model", default_value="output/temperature_model.json"),
        DeclareLaunchArgument("results", default_value="output/forecast_results.json"),
        ExecuteProcess(
            cmd=[sys.executable, str(executable), "--csv", LaunchConfiguration("csv"), "--horizon", LaunchConfiguration("horizon"), "--model", LaunchConfiguration("model"), "--results", LaunchConfiguration("results")],
            output="screen",
        ),
    ])
