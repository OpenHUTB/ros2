"""launch 入口：ros2 launch mujoco_slam_nav_sync main.launch.py"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mujoco_slam_nav_sync")
    params = PathJoinSubstitution([pkg, "config", "params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="maze",
                              description="maze（有隔断）或 open（开阔场地）"),
        DeclareLaunchArgument("use_slam", default_value="true",
                              description="true=EKF-SLAM，false=纯里程计航迹推算"),
        DeclareLaunchArgument("slip", default_value="0.0", description="轮滑率，0.0~0.3"),
        DeclareLaunchArgument("weights", default_value="",
                              description="训练好的 planner.npz 路径（空则用未训练网络）"),
        Node(
            package="mujoco_slam_nav_sync",
            executable="slam_nav_node",
            name="mujoco_slam_nav_sync",
            output="screen",
            parameters=[
                params,
                {"scene": LaunchConfiguration("scene"),
                 "use_slam": LaunchConfiguration("use_slam"),
                 "slip": LaunchConfiguration("slip"),
                 "weights": LaunchConfiguration("weights")},
            ],
        ),
    ])
