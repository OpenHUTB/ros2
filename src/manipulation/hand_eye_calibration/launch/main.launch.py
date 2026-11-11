"""通过 ROS 2 launch 启动机械臂手眼标定仿真。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hand_eye_calibration",
                executable="hand_eye_calibration",
                name="hand_eye_calibration",
                output="screen",
            )
        ]
    )