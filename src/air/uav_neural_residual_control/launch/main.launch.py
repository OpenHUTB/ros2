#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动：仿真服务端（MuJoCo）+ 神经残差控制节点。

    ros2 launch uav_neural_residual_control main.launch.py
    ros2 launch uav_neural_residual_control main.launch.py payload:=extra wind:=windy
    ros2 launch uav_neural_residual_control main.launch.py controller:=geo_rls traj:=figure8

仿真侧需要 Python ≥ 3.10（MuJoCo 3.12 的硬性要求），ROS 侧是 3.8，因此
``sim_python`` 要指向一个装了 mujoco 的解释器；两侧都装在同一个解释器时
直接留空即可。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_neural_residual_control")
    cfg = os.path.join(share, "config", "residual.yaml")

    args = [
        DeclareLaunchArgument("scene", default_value="open_ground"),
        DeclareLaunchArgument("payload", default_value="heavy",
                              description="none / light / heavy / extra"),
        DeclareLaunchArgument("wind", default_value="windy",
                              description="calm / breeze / windy"),
        DeclareLaunchArgument("ground_effect", default_value="0.12"),
        DeclareLaunchArgument("controller", default_value="geo_rls"),
        DeclareLaunchArgument("traj", default_value="helix"),
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="41460"),
        DeclareLaunchArgument("nominal_mass", default_value="0.95"),
        DeclareLaunchArgument("net", default_value="",
                              description="残差网络 .npz；留空则纯几何控制"),
        DeclareLaunchArgument("sim_python", default_value="",
                              description="仿真侧解释器（需要 mujoco），留空=当前 python3"),
        DeclareLaunchArgument("start_sim", default_value="true",
                              description="false 表示仿真已在别处运行（如宿主机）"),
        DeclareLaunchArgument("headless", default_value="false"),
    ]

    sim_python = LaunchConfiguration("sim_python")
    start_sim = LaunchConfiguration("start_sim")

    sim = ExecuteProcess(
        cmd=[PythonExpression(["'", sim_python, "' if '", sim_python,
                               "' else 'python3'"]),
             "-m", "uav_neural_residual_control.sim.plant_server",
             "--scene", LaunchConfiguration("scene"),
             "--payload", LaunchConfiguration("payload"),
             "--wind", LaunchConfiguration("wind"),
             "--ground-effect", LaunchConfiguration("ground_effect"),
             "--port", LaunchConfiguration("port")],
        output="screen",
        condition=IfCondition(start_sim),
    )
    sim_headless = ExecuteProcess(
        cmd=[PythonExpression(["'", sim_python, "' if '", sim_python,
                               "' else 'python3'"]),
             "-m", "uav_neural_residual_control.sim.plant_server",
             "--scene", LaunchConfiguration("scene"),
             "--payload", LaunchConfiguration("payload"),
             "--wind", LaunchConfiguration("wind"),
             "--ground-effect", LaunchConfiguration("ground_effect"),
             "--port", LaunchConfiguration("port"), "--headless"],
        output="screen",
        condition=IfCondition(PythonExpression(
            ["'", start_sim, "' == 'true' and '", LaunchConfiguration("headless"),
             "' == 'true'"])),
    )
    sim_visible = ExecuteProcess(
        cmd=[PythonExpression(["'", sim_python, "' if '", sim_python,
                               "' else 'python3'"]),
             "-m", "uav_neural_residual_control.sim.plant_server",
             "--scene", LaunchConfiguration("scene"),
             "--payload", LaunchConfiguration("payload"),
             "--wind", LaunchConfiguration("wind"),
             "--ground-effect", LaunchConfiguration("ground_effect"),
             "--port", LaunchConfiguration("port")],
        output="screen",
        condition=IfCondition(PythonExpression(
            ["'", start_sim, "' == 'true' and '", LaunchConfiguration("headless"),
             "' != 'true'"])),
    )

    node = Node(
        package="uav_neural_residual_control",
        executable="residual_node",
        name="uav_residual_control",
        output="screen",
        parameters=[
            cfg,
            {"host": LaunchConfiguration("host"),
             "port": LaunchConfiguration("port"),
             "controller": LaunchConfiguration("controller"),
             "trajectory": LaunchConfiguration("traj"),
             "net": LaunchConfiguration("net"),
             "nominal_mass": LaunchConfiguration("nominal_mass")},
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg=["启动仿真：载荷=", LaunchConfiguration("payload"),
                     " 风=", LaunchConfiguration("wind"),
                     " 场景=", LaunchConfiguration("scene")]),
        sim_headless,
        sim_visible,
        # 等仿真把端口监听起来；节点自身也有 wait_for_server 重试，这里只是省点日志
        TimerAction(period=2.0, actions=[node]),
    ])
