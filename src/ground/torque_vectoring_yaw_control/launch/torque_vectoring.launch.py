"""启动扭矩矢量节点。

用法::

    ros2 launch torque_vectoring_yaw_control torque_vectoring.launch.py
    ros2 launch torque_vectoring_yaw_control torque_vectoring.launch.py enable_tv:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("mu", default_value="0.95", description="路面附着系数"),
        DeclareLaunchArgument("enable_tv", default_value="true",
                              description="是否启用扭矩矢量（false 为对照组）"),
    ]
    return LaunchDescription(
        args
        + [
            Node(
                package="torque_vectoring_yaw_control",
                executable="torque_vectoring_node",
                name="torque_vectoring_node",
                output="screen",
                parameters=[{
                    "mu": LaunchConfiguration("mu"),
                    "enable_tv": LaunchConfiguration("enable_tv"),
                }],
            )
        ]
    )
