"""通过 ROS 2 launch 启动无人机自主充电/机巢演示。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="drone_charging_nest",
                executable="drone_charging_nest",
                name="drone_charging_nest",
                output="screen",
            )
        ]
    )