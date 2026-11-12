"""launch 入口：ros2 launch mujoco_mppi_neural_tracker main.launch.py"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mujoco_mppi_neural_tracker")
    params = PathJoinSubstitution([pkg, "config", "params.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="nn",
                              description="nn / mppi / pp"),
        DeclareLaunchArgument("weights", default_value="",
                              description="蒸馏网络权重；空则用包内 models/policy.npz"),
        Node(
            package="mujoco_mppi_neural_tracker",
            executable="mppi_tracker_node",
            name="mujoco_mppi_neural_tracker",
            output="screen",
            parameters=[params,
                        {"controller": LaunchConfiguration("controller"),
                         "weights": LaunchConfiguration("weights")}],
        ),
    ])
