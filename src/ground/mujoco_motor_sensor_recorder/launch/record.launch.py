"""Run the independent CSV sensor recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    package = "mujoco_motor_sensor_recorder"
    executable = Path(get_package_prefix(package)) / "lib" / package / "motor_sensor_record"
    sample_csv = Path(get_package_share_directory(package)) / "config" / "motor_state.csv"
    return LaunchDescription([
        DeclareLaunchArgument("input", default_value=str(sample_csv), description="输入电机状态 CSV 文件路径"),
        DeclareLaunchArgument("output", default_value="output/motor_sensors.csv", description="输出观测 CSV 文件路径"),
        DeclareLaunchArgument("seed", default_value="7", description="可复现的随机种子"),
        ExecuteProcess(
            cmd=[sys.executable, str(executable), "--input", LaunchConfiguration("input"), "--output", LaunchConfiguration("output"), "--seed", LaunchConfiguration("seed")],
            output="screen",
        ),
    ])
