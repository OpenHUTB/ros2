"""Gazebo 差速驱动小车 —— ROS 2 launch 文件。

用法：
    ros2 launch gazebo_diff_drive_car main.launch.py
    ros2 launch gazebo_diff_drive_car main.launch.py mode:=laser
    ros2 launch gazebo_diff_drive_car main.launch.py mode:=camera gui:=false
    ros2 launch gazebo_diff_drive_car main.launch.py mode:=teleop rviz:=true

参数：
    mode   运行模式：teleop | laser | camera | idle（默认 teleop）
    gui    是否启动 Gazebo 图形界面（默认 true）
    rviz   是否启动 RViz（默认 false）
    x y z 机器人初始位置（默认 -5.0 -4.0 0.15）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构造 launch 描述。"""
    pkg_share = get_package_share_directory('gazebo_diff_drive_car')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg_share, 'urdf', 'car.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'car_world.world')
    rviz_file = os.path.join(pkg_share, 'rviz', 'car.rviz')

    mode = LaunchConfiguration('mode')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # 直接读取 URDF 文本，避免依赖 xacro 可执行文件
    # （部分 ROS 2 环境未安装 ros-humble-xacro）
    with open(urdf_file, 'r', encoding='utf-8') as fp:
        robot_description = fp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='teleop',
            description='运行模式：teleop | laser | camera | idle'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否启动 Gazebo 图形界面'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时间'),
        DeclareLaunchArgument('x', default_value='-5.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # 启动 Gazebo 并加载场景
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': world_file,
                'gui': gui,
            }.items(),
        ),

        # 机器人描述与状态发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # 在 Gazebo 中生成机器人
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'gazebo_diff_drive_car',
                '-x', x, '-y', y, '-z', z, '-Y', yaw,
            ],
            output='screen',
        ),

        # 统一入口节点（按 mode 调度）
        Node(
            package='gazebo_diff_drive_car',
            executable='car_main',
            name='car_main',
            output='screen',
            parameters=[{
                'mode': mode,
                'use_sim_time': use_sim_time,
            }],
        ),

        # 可选的 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
