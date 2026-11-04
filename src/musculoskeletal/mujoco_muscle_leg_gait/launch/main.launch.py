"""ROS 2 launch：按场景启动一次实验。

    ros2 launch mujoco_muscle_leg_gait main.launch.py scene:=l5

说明：本模块不依赖 ROS 运行（核心是 MuJoCo + numpy），
launch 只是把 main.py 包装成 ROS 2 的启动入口，方便与其他节点统一管理。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _run(context):
    scene = LaunchConfiguration("scene").perform(context)
    share = get_package_share_directory("mujoco_muscle_leg_gait")
    return [
        ExecuteProcess(
            cmd=["python3", os.path.join(share, "main.py"), "run", "--scene", scene],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene", default_value="l5",
            description="实验编号：l1 / l2 / l3 / l4 / l5 / l6",
        ),
        OpaqueFunction(function=_run),
    ])
