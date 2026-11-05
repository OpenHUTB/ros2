#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_mujoco_sim 统一启动入口：仿真服务端 + 桥接节点 + 键盘遥控节点（可选，默认关）。

    ros2 launch uav_mujoco_sim main.launch.py [scene:=… quality:=full]

仿真与 ROS2 分属两个进程：rclpy 只有 Python 3.8 绑定，MuJoCo 3.12 需 Python ≥3.10。
键盘遥控要独占一个前台终端，另开一个终端跑 `ros2 run uav_mujoco_sim keyboard`。
"""
import os
import shutil
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _usable(python):
    """该解释器能否 import mujoco 且版本 ≥3.12。"""
    if not python:
        return False
    if not (os.path.isabs(python) and os.path.exists(python)) and \
            shutil.which(python) is None:
        return False
    code = ("import mujoco,sys;"
            "v=tuple(int(x) for x in mujoco.__version__.split('.')[:2]);"
            "sys.exit(0 if v>=(3,12) else 1)")
    try:
        r = subprocess.run([python, "-c", code], capture_output=True, timeout=60)
        return r.returncode == 0
    except Exception:
        return False


def _find_sim_python(explicit):
    """按优先级找一个装了 mujoco>=3.12 的 Python。"""
    if explicit:
        return explicit if _usable(explicit) else None
    home = os.path.expanduser("~")
    cands = [
        os.path.join(home, "uav_ws", ".venv", "bin", "python"),   # 本项目推荐位置
        os.path.join(home, ".venv", "bin", "python"),
        "python3.12", "python3.11", "python3.10", "python3",
    ]
    for c in cands:
        if _usable(c):
            return c
    return None


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("uav_mujoco_sim")
    cfg = os.path.join(pkg_share, "config", "sim.yaml")

    lip = LaunchConfiguration
    explicit = lip("sim_python").perform(context)
    sim_python = _find_sim_python(explicit)

    if sim_python is None:
        raise RuntimeError(
            "\n" + "=" * 74 + "\n"
            "找不到可用的仿真侧 Python（需要 python>=3.10 且已安装 mujoco>=3.12）。\n"
            "请先执行：\n"
            "    curl -LsSf https://astral.sh/uv/install.sh | sh\n"
            "    uv venv ~/uav_ws/.venv --python 3.12\n"
            "    uv pip install --python ~/uav_ws/.venv/bin/python "
            "'mujoco==3.12.0' numpy\n"
            "或显式指定：sim_python:=/path/to/python\n"
            "=" * 74)
    print(f"[uav_mujoco_sim] 仿真侧解释器: {sim_python}")

    # 源码目录（开发）与安装后的 share 目录都可运行
    src_dir = os.path.join(pkg_share, "..", "..", "..", "..", "src",
                           "uav_mujoco_sim")
    main_py = os.path.join(pkg_share, "main.py")
    if not os.path.exists(main_py):
        main_py = os.path.join(os.path.dirname(__file__), "..", "main.py")

    server_cmd = [
        sim_python, os.path.abspath(main_py), "--server",
        "--host", lip("sim_host").perform(context),
        "--port", lip("sim_port").perform(context),
        "--scene", lip("scene").perform(context),
        "--seed", lip("seed").perform(context),
        "--dt", lip("dt").perform(context),
        "--width", lip("width").perform(context),
        "--height", lip("height").perform(context),
        "--quality", lip("quality").perform(context),
    ]
    if lip("headless").perform(context).lower() == "true":
        server_cmd.append("--headless")
    if lip("imu_noise").perform(context).lower() == "true":
        server_cmd.append("--imu-noise")

    # 无显示环境改走 osmesa；headless 不建渲染器，不需要指定 GL 后端
    env = {}
    if lip("headless").perform(context).lower() != "true" and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        env["MUJOCO_GL"] = os.environ.get("MUJOCO_GL", "osmesa")

    sim_server = ExecuteProcess(
        cmd=server_cmd, name="uav_sim_server", output="screen", env=env)

    bridge_params = [cfg, {
        "sim/host": lip("sim_host"),
        "sim/port": lip("sim_port"),
        "frame/world": "world",
        "frame/body": "base_link",
    }]

    bridge = Node(
        package="uav_mujoco_sim", executable="bridge",
        name="uav_sim_bridge", output="screen", parameters=bridge_params)

    keyboard = Node(
        package="uav_mujoco_sim", executable="keyboard",
        name="uav_keyboard_teleop", output="screen",
        condition=IfCondition(lip("keyboard")))

    # 桥接自带 30 s 重试，这里只做 2 s 错峰
    return [sim_server, TimerAction(period=2.0, actions=[bridge, keyboard])]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("scene", default_value="open_ground",
                              choices=["open_ground", "obstacle_field",
                                       "corridor", "gate_track", "urban_block"],
                              description="场景"),
        DeclareLaunchArgument("seed", default_value="0", description="场景随机种子"),
        DeclareLaunchArgument("sim_python", default_value="",
                              description="仿真侧 Python 解释器（留空自动探测）"),
        DeclareLaunchArgument("sim_host", default_value="127.0.0.1",
                              description="桥接连接仿真端的地址"),
        DeclareLaunchArgument("sim_port", default_value="41460"),
        DeclareLaunchArgument("dt", default_value="0.01",
                              description="控制周期（秒）"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("quality", default_value="fast",
                              choices=["fast", "full"],
                              description="fast=关阴影(感知用,快6倍)；full=录动图用"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true 则不渲染视觉传感器"),
        DeclareLaunchArgument("imu_noise", default_value="false"),
        # 键盘节点默认不开：launch 拉起的子进程不在终端的前台进程组，读终端收不到
        # 按键也不报错。遥控要另开一个终端跑 `ros2 run uav_mujoco_sim keyboard`。
        DeclareLaunchArgument("keyboard", default_value="false",
                              description="是否同时启动任务①的键盘遥控节点"
                                          "（默认关，见 README 5.3）"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
