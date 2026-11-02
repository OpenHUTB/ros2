"""ROS 2 Humble launch file / ROS 2 Humble 启动文件。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="mujoco_abandoned_park_uav_visual_steering",
            executable="visual_steering_node",
            name="visual_steering_node",
            parameters=["config/ros_visual_steering.yaml"],
            output="screen",
        )
    ])


