"""ROS 2 launch：运行一组辨识实验。

    ros2 launch mujoco_muscle_ident main.launch.py scene:=i1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _run(context):
    scene = LaunchConfiguration("scene").perform(context)
    share = get_package_share_directory("mujoco_muscle_ident")
    return [ExecuteProcess(
        cmd=["python3", os.path.join(share, "main.py"), "run", "--scene", scene],
        output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="i1",
                              description="实验编号：i1 / i2 / i3 / i4 / i5"),
        OpaqueFunction(function=_run),
    ])
