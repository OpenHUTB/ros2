# -*- coding: utf-8 -*-
"""ROS 2 launch：拉起窄通道通行节点。

    ros2 launch humanoid_passage_width_sweep main.launch.py
    ros2 launch humanoid_passage_width_sweep main.launch.py width:=0.78 variant:=mppi_point
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("humanoid_passage_width_sweep")

    args = [
        DeclareLaunchArgument("width", default_value="0.90",
                              description="窄门净宽 (m)"),
        DeclareLaunchArgument("variant", default_value="mppi_footprint",
                              description="规划器变体"),
        DeclareLaunchArgument("publish_rate", default_value="20.0"),
        DeclareLaunchArgument("use_viewer", default_value="false",
                              description="是否另起 MuJoCo 窗口"),
    ]

    node = Node(
        package="humanoid_passage_width_sweep",
        executable="passage_sweep_node",
        name="passage_sweep_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg, "config", "params.yaml"]),
            {
                "width": LaunchConfiguration("width"),
                "variant": LaunchConfiguration("variant"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            },
        ],
    )

    # viewer 不是 ROS 节点，用命令行参数把 launch 的取值传进去，
    # 否则窗口会恒定使用 DEFAULT_WIDTH 与 mppi_footprint，与实际运行的节点不一致。
    viewer = Node(
        package="humanoid_passage_width_sweep",
        executable="passage_viewer",
        name="passage_viewer",
        output="screen",
        arguments=[
            "--width", LaunchConfiguration("width"),
            "--variant", LaunchConfiguration("variant"),
        ],
        condition=IfCondition(LaunchConfiguration("use_viewer")),
    )

    return LaunchDescription(args + [node, viewer])