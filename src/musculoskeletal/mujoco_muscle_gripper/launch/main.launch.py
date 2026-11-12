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
    mode = LaunchConfiguration("mode").perform(context)
    out = LaunchConfiguration("out").perform(context)
    keys = LaunchConfiguration("keys").perform(context)
    if mode not in ("run", "bench", "neural", "manual", "check", "gif", "slam"):
        raise ValueError("未知运行模式：" + mode)
    share = get_package_share_directory("mujoco_muscle_gripper")
    args = [mode]
    if mode == "run":
        args += ["--scene", scene]
    elif mode in ("bench", "neural", "gif", "slam"):
        args += ["--out", out]
    elif mode == "manual":
        args += ["--keys", keys]
    return [
        ExecuteProcess(
            cmd=["python3", os.path.join(share, "main.py")] + args,
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="g2",
                              description="实验编号：g1 至 g8"),
        DeclareLaunchArgument("mode", default_value="run", description="run / bench / neural / manual / check / gif / slam"),
        DeclareLaunchArgument("out", default_value="/tmp/gripper-results", description="结果目录"),
        DeclareLaunchArgument("keys", default_value="wwwwwwwwwwrrrrq", description="手动模式可重复按键序列"),
        OpaqueFunction(function=_run),
    ])
