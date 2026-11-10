"""ROS 2 launch：按场景运行一次实验。

    ros2 launch mujoco_muscle_gripper main.launch.py scene:=g2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _run(context):
    scene = LaunchConfiguration("scene").perform(context)
    share = get_package_share_directory("mujoco_muscle_gripper")
    return [
        ExecuteProcess(
            cmd=["python3", os.path.join(share, "main.py"), "run", "--scene", scene],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="g2",
                              description="实验编号：g1 / g2 / g3 / g4"),
        OpaqueFunction(function=_run),
    ])
