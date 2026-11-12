# -*- coding: utf-8 -*-
"""一键启动：`ros2 launch mujoco_balance_car main.launch.py`

本模块本身不依赖 ROS（纯 mujoco + numpy）。这个 launch 的价值是把"离线批跑"
纳入 ROS 2 的统一启动方式，方便与其它模块用同一套命令验证。

所有 `<arg>` 都真正透传到进程参数里——ROS 2 对未声明的 launch 参数不报错、
直接忽略，写了不传等于"看起来能配、实际没生效"。

    ros2 launch mujoco_balance_car main.launch.py
    ros2 launch mujoco_balance_car main.launch.py duration:=8 lean:=0.15
    ros2 launch mujoco_balance_car main.launch.py save_dir:=/tmp/balance_shots
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    save_dir = LaunchConfiguration("save_dir")
    duration = LaunchConfiguration("duration")
    lean = LaunchConfiguration("lean")

    declare_save_dir = DeclareLaunchArgument(
        "save_dir", default_value="shots",
        description="CSV/PNG 输出目录")
    declare_duration = DeclareLaunchArgument(
        "duration", default_value="6.0", description="单场景仿真时长（s）")
    declare_lean = DeclareLaunchArgument(
        "lean", default_value="0.05", description="初始倾角（rad）")

    run_demo = ExecuteProcess(
        cmd=["mujoco_balance_car", "--headless",
             "--save_dir", save_dir,
             "--duration", duration,
             "--lean", lean],
        output="screen",
    )

    return LaunchDescription([
        LogInfo(msg="启动 MuJoCo 两轮自平衡小车离线评测（无窗口）"),
        declare_save_dir,
        declare_duration,
        declare_lean,
        run_demo,
    ])
