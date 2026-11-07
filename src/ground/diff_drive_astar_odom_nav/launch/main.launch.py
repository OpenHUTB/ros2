#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动：Gazebo 静态场景 + 差速底盘 + 静态地图 + A* 规划 + 里程计跟踪 + 数据记录。

用法（默认无界面，适合虚拟机/CI）：
    ros2 launch diff_drive_astar_odom_nav main.launch.py
有显示器的机器上看 Gazebo 画面：
    ros2 launch diff_drive_astar_odom_nav main.launch.py gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "diff_drive_astar_odom_nav"
    share = get_package_share_directory(pkg)
    world = os.path.join(share, "worlds", "static_obstacles.world")
    urdf = os.path.join(share, "urdf", "diff_robot.urdf")
    with open(urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    gui = LaunchConfiguration("gui")
    start_gazebo = LaunchConfiguration("start_gazebo")
    output_dir = LaunchConfiguration("output_dir")
    auto_goal_x = LaunchConfiguration("auto_goal_x")
    auto_goal_y = LaunchConfiguration("auto_goal_y")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch/gzserver.launch.py"]),
        launch_arguments={"world": world, "verbose": "false"}.items(),
        condition=IfCondition(start_gazebo),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch/gzclient.launch.py"]),
        condition=IfCondition(gui),
    )
    spawn_robot = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", "diff_robot", "-file", urdf,
                   "-x", "-4.5", "-y", "-4.5", "-z", "0.02"],
        output="screen",
        condition=IfCondition(start_gazebo),
    )
    robot_state = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        output="screen",
    )
    map_publisher = Node(
        package=pkg, executable="map_publisher", name="static_map_publisher",
        parameters=[{"map_yaml": os.path.join(share, "maps", "static_map.yaml"),
                     "frame_id": "map"}],
        output="screen",
    )
    planner = Node(
        package=pkg, executable="planner", name="astar_planner",
        parameters=[{
            "robot_radius": 0.22,
            "inflation_extra": 0.05,
            "allow_diagonal": True,
            "weight": 1.0,
            "auto_goal_x": auto_goal_x,
            "auto_goal_y": auto_goal_y,
        }],
        output="screen",
    )
    tracker = Node(
        package=pkg, executable="tracker", name="odom_tracker",
        parameters=[{
            "lookahead": 0.55,
            "v_max": 0.75,
            "v_min": 0.18,
            "w_max": 1.40,
            "goal_tolerance": 0.12,
            "max_time": 90.0,
            "status_file": PathJoinSubstitution([output_dir, "status.json"]),
        }],
        output="screen",
    )
    logger = Node(
        package=pkg, executable="logger", name="experiment_logger",
        parameters=[{"output_dir": output_dir}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="是否启动 Gazebo 图形界面"),
        DeclareLaunchArgument("start_gazebo", default_value="true",
                              description="是否由本 launch 启动 gzserver"),
        DeclareLaunchArgument("output_dir", default_value="results/latest",
                              description="实验数据输出目录"),
        DeclareLaunchArgument("auto_goal_x", default_value="4.5"),
        DeclareLaunchArgument("auto_goal_y", default_value="4.5"),
        gzserver,
        gzclient,
        TimerAction(period=2.0, actions=[spawn_robot]),
        robot_state,
        map_publisher,
        planner,
        tracker,
        logger,
    ])
