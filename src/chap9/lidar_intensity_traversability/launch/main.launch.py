"""启动仿真小车、激光雷达与强度可通行性节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("max_speed", default_value="0.35"),
        DeclareLaunchArgument("stop_distance", default_value="0.45"),
        DeclareLaunchArgument("uncertainty", default_value="0.60"),
        DeclareLaunchArgument("csv_path", default_value="/tmp/lidar_intensity_demo.csv"),
    ]

    sim = Node(
        package="lidar_intensity_traversability",
        executable="scan_sim",
        name="scan_sim",
        output="screen",
        parameters=[{"csv_path": LaunchConfiguration("csv_path")}],
    )
    controller = Node(
        package="lidar_intensity_traversability",
        executable="main",
        name="intensity_traversability",
        output="screen",
        parameters=[{
            "max_speed": LaunchConfiguration("max_speed"),
            "stop_distance": LaunchConfiguration("stop_distance"),
            "uncertainty": LaunchConfiguration("uncertainty"),
        }],
    )
    return LaunchDescription(args + [sim, controller])
