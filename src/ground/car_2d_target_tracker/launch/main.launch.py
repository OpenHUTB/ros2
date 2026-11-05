# -*- coding: utf-8 -*-
"""一键启动：`ros2 launch car_2d_target_tracker main.launch.py`

默认走离线批跑（不需要 pygame / 图形界面）；所有 <arg> 都真正拼进进程参数里。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    sim_time = LaunchConfiguration("sim_time")
    save_dir = LaunchConfiguration("save_dir")

    declare_mode = DeclareLaunchArgument(
        "mode", default_value="both", description="对照控制器：both | baseline | escape")
    declare_sim_time = DeclareLaunchArgument(
        "sim_time", default_value="30.0", description="单场景时长 (s)")
    declare_save_dir = DeclareLaunchArgument(
        "save_dir", default_value="shots", description="CSV/PNG 输出目录")

    run_demo = ExecuteProcess(
        cmd=["car_2d_target_tracker", "--headless",
             "--mode", mode,
             "--sim-time", sim_time,
             "--save-dir", save_dir],
        output="screen",
    )

    return LaunchDescription([
        LogInfo(msg="启动 2D 小车目标追踪离线评测（无窗口）"),
        declare_mode,
        declare_sim_time,
        declare_save_dir,
        run_demo,
    ])
