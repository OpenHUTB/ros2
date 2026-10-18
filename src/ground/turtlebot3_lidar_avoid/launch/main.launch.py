"""TurtleBot3 激光雷达避障 —— ROS 2 launch 文件。

用法：
    ros2 launch turtlebot3_lidar_avoid main.launch.py
    ros2 launch turtlebot3_lidar_avoid main.launch.py scan_topic:=/out/scan

说明：
    本 launch 只负责启动避障节点，Gazebo 仿真环境需另行启动，例如：
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    或使用课程镜像自带脚本：
        ~/start-gazebo-turtlebot-ros-humble.sh
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构造 launch 描述。"""
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='激光雷达话题。课程镜像的 TurtleBot3 迷宫场景可能为 /out/scan'),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='速度指令话题'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='可选：参数 YAML 文件路径'),

        Node(
            package='turtlebot3_lidar_avoid',
            executable='avoid_node',
            name='turtlebot3_lidar_avoid',
            output='screen',
            parameters=[{
                'scan_topic': scan_topic,
                'cmd_vel_topic': cmd_vel_topic,
            }],
        ),
    ])
