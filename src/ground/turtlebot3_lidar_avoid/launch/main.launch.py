"""TurtleBot3 激光雷达避障 —— ROS 2 launch 文件。

用法：
    ros2 launch turtlebot3_lidar_avoid main.launch.py
    ros2 launch turtlebot3_lidar_avoid main.launch.py scan_topic:=/out/scan

说明：
    本 launch 会自动设置 TURTLEBOT3_MODEL=burger，无需手动 export。
    若需另行启动 Gazebo 仿真环境，新开终端执行：
        export TURTLEBOT3_MODEL=burger
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    （若使用预装特定课程环境的虚拟机镜像，也可使用其提供的快捷启动脚本 ~/start-gazebo-turtlebot-ros-humble.sh）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构造 launch 描述。"""
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    model = LaunchConfiguration('model')

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
            'model',
            default_value='burger',
            description='TurtleBot3 型号：burger / waffle / waffle_pi'),

        # turtlebot3_gazebo / turtlebot3 系列包启动时会读这个环境变量，
        # 未设置会直接 KeyError: 'TURTLEBOT3_MODEL'
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),

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
