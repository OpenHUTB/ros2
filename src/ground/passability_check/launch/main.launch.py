# -*- coding: utf-8 -*-
"""一键启动通过性核算节点。

用法：
    ros2 launch passability_check main.launch.py
    ros2 launch passability_check main.launch.py vehicle:=suv
    ros2 launch passability_check main.launch.py vehicle:=suv scenario:=mountain_crest
    ros2 launch passability_check main.launch.py scenario_file:=/abs/path/my_ramp.json
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "passability_check"


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(package_share, "config", "params.yaml")

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="ROS 参数文件（车型、场景、采样密度、发布频率）",
    )
    declare_vehicle = DeclareLaunchArgument(
        "vehicle", default_value="sedan",
        description="车型：sedan / suv",
    )
    declare_scenario = DeclareLaunchArgument(
        "scenario", default_value="garage_ramp",
        description="场景名：garage_ramp / mountain_crest / speed_bump",
    )
    declare_scenario_file = DeclareLaunchArgument(
        "scenario_file", default_value="",
        description="直接用某个场景 JSON 的绝对路径（非空时优先于 scenario）",
    )
    declare_samples = DeclareLaunchArgument(
        "samples", default_value="41",
        description="车身下缘采样点数",
    )
    declare_steps = DeclareLaunchArgument(
        "steps", default_value="480",
        description="沿剖面扫描的位姿档数",
    )
    declare_hz = DeclareLaunchArgument(
        "publish_hz", default_value="1.0",
        description="报告重发频率（Hz）",
    )

    node = Node(
        package=PACKAGE_NAME,
        executable="passability_check_node",
        name="passability_check",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"), {
            "vehicle": LaunchConfiguration("vehicle"),
            "scenario": LaunchConfiguration("scenario"),
            "scenario_file": LaunchConfiguration("scenario_file"),
            "samples": LaunchConfiguration("samples"),
            "steps": LaunchConfiguration("steps"),
            "publish_hz": LaunchConfiguration("publish_hz"),
        }],
    )

    return LaunchDescription([
        declare_params,
        declare_vehicle,
        declare_scenario,
        declare_scenario_file,
        declare_samples,
        declare_steps,
        declare_hz,
        node,
    ])
