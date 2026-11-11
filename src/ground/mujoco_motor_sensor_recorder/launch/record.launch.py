"""Run the independent CSV sensor recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix


def generate_launch_description() -> LaunchDescription:
    executable = Path(get_package_prefix("mujoco_motor_sensor_recorder")) / "lib" / "mujoco_motor_sensor_recorder" / "motor_sensor_record"
    return LaunchDescription([
        DeclareLaunchArgument("input"),
        DeclareLaunchArgument("output", default_value="output/motor_sensors.csv"),
        ExecuteProcess(
            cmd=[sys.executable, str(executable), "--input", LaunchConfiguration("input"), "--output", LaunchConfiguration("output")],
            output="screen",
        ),
    ])
