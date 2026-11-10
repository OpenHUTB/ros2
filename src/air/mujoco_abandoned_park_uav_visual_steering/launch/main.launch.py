"""Installed ROS 2 image steering launch / 安装后的 ROS 2 图像转向启动入口。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("mujoco_abandoned_park_uav_visual_steering"))
    default_checkpoint = share / "models" / "weights" / "lightweight_steering_cnn.pt"
    return LaunchDescription([
        DeclareLaunchArgument("checkpoint", default_value=str(default_checkpoint), description="CNN checkpoint / CNN 权重路径"),
        Node(
            package="mujoco_abandoned_park_uav_visual_steering",
            executable="visual_steering_node",
            name="visual_steering_node",
            parameters=[str(share / "config" / "ros_visual_steering.yaml"),
                        {"checkpoint": ParameterValue(LaunchConfiguration("checkpoint"), value_type=str)}],
            output="screen",
        ),
    ])
