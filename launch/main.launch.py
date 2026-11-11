"""ROS 2 Humble launch file / ROS 2 Humble 启动文件。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("checkpoint", default_value="", description="CNN checkpoint / CNN 权重路径"),
        Node(
            package="mujoco_abandoned_park_uav_visual_steering",
            executable="visual_steering_node",
            name="visual_steering_node",
            parameters=[str(Path(get_package_share_directory("mujoco_abandoned_park_uav_visual_steering"))
                            / "config" / "ros_visual_steering.yaml"),
                        {"checkpoint": ParameterValue(LaunchConfiguration("checkpoint"), value_type=str)}],
            output="screen",
        )
    ])


