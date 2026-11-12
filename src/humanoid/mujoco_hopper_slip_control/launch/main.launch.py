# -*- coding: utf-8 -*-
"""ROS 2 launch：一条命令拉起单足弹跳。

    ros2 launch mujoco_hopper_slip_control main.launch.py
    ros2 launch mujoco_hopper_slip_control main.launch.py target_height:=0.62
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("mujoco_hopper_slip_control")

    args = [
        DeclareLaunchArgument("target_height", default_value="0.55",
                              description="躯干顶点目标高度 (m)"),
        DeclareLaunchArgument("target_speed", default_value="0.0",
                              description="目标前进速度 (m/s)，0 为原地跳"),
        DeclareLaunchArgument("control_hz", default_value="100.0",
                              description="控制定时器频率 (Hz)"),
        DeclareLaunchArgument("publish_rate", default_value="50.0",
                              description="状态发布频率 (Hz)"),
        # 有图形环境时可设为 true 另开 MuJoCo 窗口；无显示器（CI/容器）保持 false
        DeclareLaunchArgument("use_viewer", default_value="false",
                              description="是否另起 mujoco.viewer 窗口"),
    ]

    node = Node(
        package="mujoco_hopper_slip_control",
        executable="hopper_slip_node",
        name="hopper_slip_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg, "config", "params.yaml"]),
            {
                "target_height": LaunchConfiguration("target_height"),
                "target_speed": LaunchConfiguration("target_speed"),
                "control_hz": LaunchConfiguration("control_hz"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            },
        ],
    )

    # viewer 不是 ROS 节点，用命令行参数把 launch 的取值传进去，
    # 避免窗口恒用 config 默认值、与节点实际参数不一致。
    viewer = Node(
        package="mujoco_hopper_slip_control",
        executable="hopper_viewer",
        name="hopper_viewer",
        output="screen",
        arguments=[
            "--target-height", LaunchConfiguration("target_height"),
            "--target-speed", LaunchConfiguration("target_speed"),
        ],
        condition=IfCondition(LaunchConfiguration("use_viewer")),
    )

    return LaunchDescription(args + [node, viewer])
