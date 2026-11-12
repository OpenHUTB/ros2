import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_diff_drive_teleop')
    xacro_path = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    demo = LaunchConfiguration('demo')
    gz_args = LaunchConfiguration('gz_args')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # ros_gz_sim 不会自动桥接 Gazebo 的 /clock；控制器启用了
    # use_sim_time，没有时钟就不会更新里程计。
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_diff_drive_robot',
        output='screen',
        arguments=[
            '-name', 'diff_drive_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )

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

    # Jazzy 的控制器使用 TwistStamped 原生话题；这里与标准
    # /cmd_vel、/odom 互转，键盘和自动巡航节点无需感知控制器细节。
    controller_bridge = Node(
        package='gazebo_diff_drive_teleop',
        executable='controller_bridge',
        name='controller_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    auto_drive = Node(
        package='gazebo_diff_drive_teleop',
        executable='auto_drive',
        name='auto_drive',
        output='screen',
        condition=IfCondition(demo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'demo', default_value='true',
            description='true 时启动自动巡航演示；teach 模式设为 false'),
        DeclareLaunchArgument(
            'gz_args', default_value='-r',
            description='传给 Gazebo 的参数；加 -s 可只启动无界面服务端'),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        diff_drive_controller,
        controller_bridge,
        auto_drive,
    ])
