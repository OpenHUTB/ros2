# -*- coding: utf-8 -*-
"""一键启动 ROS 2 闭环：MuJoCo 仿真桥接 + 翻车风险预测与主动限速。

sim_bridge 在 ROS 里把 MuJoCo 小车真实跑起来（发布 /imu、/odom、/chassis_state，
订阅 /cmd_vel）；guard_node 用训练好的 MLP 预测 0.5s 内翻车概率，按风险限速后
把指令发回 /cmd_vel。桥接节点跑完本工况退出后，launch 随机关闭。

用法：
    ros2 launch mujoco_car_rollover_guard main.launch.py
    ros2 launch mujoco_car_rollover_guard main.launch.py scene:=car_scene_payload_low.xml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "mujoco_car_rollover_guard"


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(package_share, "config", "params.yaml")

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="ROS 参数文件（场景、目标车速、转向斜率、运行时长）",
    )

    declare_scene = DeclareLaunchArgument(
        "scene",
        default_value="car_scene_payload_mid.xml",
        description="运行场景 XML（载荷高度分档）",
    )
    declare_v = DeclareLaunchArgument(
        "v_target",
        default_value="1.4",
        description="目标车速 (m/s)",
    )
    declare_ramp = DeclareLaunchArgument(
        "steer_ramp",
        default_value="0.45",
        description="转向指令每秒增量（越大越早翻车）",
    )
    declare_seconds = DeclareLaunchArgument(
        "run_seconds",
        default_value="9.0",
        description="单次工况最长运行时间 (s)",
    )

    bridge = Node(
        package=PACKAGE_NAME,
        executable="mujoco_car_rollover_sim_bridge_node",
        name="mujoco_car_rollover_sim_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"), {
            "scene": LaunchConfiguration("scene"),
            "run_seconds": LaunchConfiguration("run_seconds"),
        }],
    )
    guard = Node(
        package=PACKAGE_NAME,
        executable="mujoco_car_rollover_guard_node",
        name="mujoco_car_rollover_guard",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"), {
            "v_target": LaunchConfiguration("v_target"),
            "steer_ramp": LaunchConfiguration("steer_ramp"),
            "run_seconds": LaunchConfiguration("run_seconds"),
            "model_path": os.path.join(package_share, "results", "rollover_risk.npz"),
        }],
    )

    return LaunchDescription([
        declare_params,
        declare_scene,
        declare_v,
        declare_ramp,
        declare_seconds,
        bridge,
        guard,
        # 桥接节点跑完当前工况退出后，整个闭环随之关闭，避免空转
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=Shutdown())),
    ])
