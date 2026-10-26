# -*- coding: utf-8 -*-
"""一键启动 ROS 2 闭环：MuJoCo 仿真桥接 + 神经网络避障推理。

sim_bridge 在 ROS 里把 MuJoCo 小车真实跑起来（发布 /scan、/odom，订阅 /cmd_vel）；
ros_node 收到 /scan 用训练好的 MLP 推理，发布 /cmd_vel 反驱动小车。桥接节点跑完
三个场景、画完真实轨迹后退出，launch 随机关闭。

用法：
    ros2 launch mujoco_car_nn_avoidance main.launch.py
    ros2 launch mujoco_car_nn_avoidance main.launch.py params_file:=<自定义参数文件路径>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "mujoco_car_nn_avoidance"


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(package_share, "config", "params.yaml")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="ROS 参数文件（场景列表、目标距离、超时时间）",
    )

    bridge = Node(
        package=PACKAGE_NAME,
        executable="mujoco_car_nn_sim_bridge_node",
        name="mujoco_car_nn_sim_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )
    agent = Node(
        package=PACKAGE_NAME,
        executable="mujoco_car_nn_avoidance_node",
        name="mujoco_car_nn_avoidance",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_params_file,
        bridge,
        agent,
        # 桥接节点跑完三个场景退出后，整个闭环随之关闭，避免空转
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=Shutdown())),
    ])
