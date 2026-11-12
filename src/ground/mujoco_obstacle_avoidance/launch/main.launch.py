# -*- coding: utf-8 -*-
"""ROS 2 启动文件：拉起 MuJoCo 差速小车避障仿真。

用法：
    ros2 launch mujoco_obstacle_avoidance main.launch.py
    ros2 launch mujoco_obstacle_avoidance main.launch.py headless:=true
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = "mujoco_obstacle_avoidance"


def generate_launch_description():
    # install 空间：入口脚本装在 lib/<pkg>/main.py；未安装时回退到源码包根。
    try:
        pkg_prefix = get_package_prefix(PACKAGE_NAME)
        main_script = os.path.join(pkg_prefix, "lib", PACKAGE_NAME, "main.py")
    except Exception:
        main_script = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "main.py")

    headless = LaunchConfiguration("headless")

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="是否无头运行（true/false）"),
        ExecuteProcess(
            cmd=["python3", main_script],
            output="screen",
            condition=UnlessCondition(headless),
        ),
        ExecuteProcess(
            cmd=["python3", main_script, "--headless"],
            output="screen",
            condition=IfCondition(headless),
        ),
    ])
