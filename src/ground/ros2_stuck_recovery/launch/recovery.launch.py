"""启动文件：把卡死检测与脱困层插到任务层与底盘之间。

用法::

    ros2 launch ros2_stuck_recovery recovery.launch.py
    ros2 launch ros2_stuck_recovery recovery.launch.py task_topic:=/cmd_vel_nav cmd_topic:=/cmd_vel

注意：``cmd_topic`` 必须接到**底盘驱动**订阅的话题，而任务层要改发到 ``task_topic``，
否则会形成自环（本节点订阅自己的输出）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("odom_topic", default_value="/odom", description="轮式里程计话题"),
        DeclareLaunchArgument("imu_topic", default_value="/imu", description="IMU 话题"),
        DeclareLaunchArgument("scan_topic", default_value="/scan", description="激光雷达话题"),
        DeclareLaunchArgument("task_topic", default_value="/cmd_vel_nav", description="任务层速度指令"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel", description="底盘速度指令"),
        DeclareLaunchArgument("rate_hz", default_value="20.0", description="控制频率"),
        DeclareLaunchArgument("max_level", default_value="3", description="最高脱困级别"),
    ]

    node = Node(
        package="ros2_stuck_recovery",
        executable="stuck_recovery_node",
        name="stuck_recovery",
        output="screen",
        parameters=[
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "task_topic": LaunchConfiguration("task_topic"),
                "cmd_topic": LaunchConfiguration("cmd_topic"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "max_level": LaunchConfiguration("max_level"),
            }
        ],
    )

    return LaunchDescription(args + [node])
