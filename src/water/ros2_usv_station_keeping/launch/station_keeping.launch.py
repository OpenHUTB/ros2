"""启动定点保持节点。

用法::

    ros2 launch ros2_usv_station_keeping station_keeping.launch.py
    ros2 launch ros2_usv_station_keeping station_keeping.launch.py target_x:=2.0 target_y:=-1.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("target_x", default_value="0.0", description="目标点 x（世界系，m）"),
        DeclareLaunchArgument("target_y", default_value="0.0", description="目标点 y（世界系，m）"),
        DeclareLaunchArgument("odom_topic", default_value="/odom", description="里程计话题"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel", description="底盘速度指令话题"),
        DeclareLaunchArgument("rate_hz", default_value="20.0", description="控制频率"),
    ]
    return LaunchDescription(
        args
        + [
            Node(
                package="ros2_usv_station_keeping",
                executable="station_keeping_node",
                name="station_keeping_node",
                output="screen",
                parameters=[
                    {
                        "target_x": LaunchConfiguration("target_x"),
                        "target_y": LaunchConfiguration("target_y"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "cmd_topic": LaunchConfiguration("cmd_topic"),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                    }
                ],
            )
        ]
    )
