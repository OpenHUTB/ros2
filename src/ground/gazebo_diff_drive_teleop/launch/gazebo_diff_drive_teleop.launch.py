import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_diff_drive_teleop')
    xacro_path = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.urdf.xacro')

    # 用 xacro 展开 URDF（含 $(find ...) 与 ros2_control 硬件接口）
    robot_description = Command(['xacro ', xacro_path])

    # demo=true 时启动自动巡航节点，false 时仅启动仿真（配合键盘控制）
    demo = LaunchConfiguration('demo', default='true')

    # 启动 Gazebo Harmonic（空世界）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        )
    )

    # 发布机器人模型与静态 TF（base_footprint -> base_link -> wheels/caster）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 将机器人生成到 Gazebo 中（gz_ros2_control 插件随模型一起加载）
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_diff_drive_robot',
        output='screen',
        arguments=['-name', 'diff_drive_robot',
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
    )

    # 加载并启动控制器
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # 自动巡航演示节点（可选）
    auto_drive = Node(
        package='gazebo_diff_drive_teleop',
        executable='auto_drive',
        name='auto_drive',
        output='screen',
        condition=IfCondition(demo),
    )

    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='true',
                              description='true 时启动自动巡航演示，false 时仅启动仿真'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        diff_drive_controller,
        auto_drive,
    ])
