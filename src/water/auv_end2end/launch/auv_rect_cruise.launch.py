import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('auv_end2end')
    world_path = os.path.join(share_dir, 'worlds', 'underwater_cruise.world')
    rviz_config = os.path.join(share_dir, 'rviz', 'auv.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'forward_speed', default_value='2.5',
            description='前进/后退速度（m/s）'),
        DeclareLaunchArgument(
            'side_speed', default_value='2.0',
            description='横向平移速度（m/s）'),
        DeclareLaunchArgument(
            'forward_time', default_value='10.0',
            description='直行/后退持续时间（s）'),
        DeclareLaunchArgument(
            'side_time', default_value='20.0',
            description='横向平移持续时间（s）'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='true 时启动 RViz2 显示 /auv/trajectory 轨迹'),
        DeclareLaunchArgument(
            'record_csv', default_value='',
            description='轨迹 CSV 输出路径，留空则不写文件'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='true 时启动 gzclient 图形界面，无显示环境请设为 false'),
        # 启动Gazebo仿真（空场景，避免墙体切断矩形轨迹）
        # gazebo_ros_init 是 SystemPlugin，必须在 gzserver 启动时用 -s 加载，
        # 只有这样才会发布 /clock，use_sim_time 的节点才不会停在 0 时刻
        ExecuteProcess(
            cmd=['gzserver', '--verbose',
                 '-s', 'libgazebo_ros_init.so', world_path],
            output='screen',
            name='gzserver'
        ),
        # gzclient 单独启动：没有 DISPLAY 时只会关掉界面，仿真本身继续运行
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        # 启动矩形巡航节点
        Node(
            package='auv_end2end',
            executable='auv_rect_cruise',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'forward_speed': ParameterValue(
                    LaunchConfiguration('forward_speed'), value_type=float),
                'side_speed': ParameterValue(
                    LaunchConfiguration('side_speed'), value_type=float),
                'forward_time': ParameterValue(
                    LaunchConfiguration('forward_time'), value_type=float),
                'side_time': ParameterValue(
                    LaunchConfiguration('side_time'), value_type=float),
            }]
        ),
        # 轨迹记录：发布 /auv/trajectory 供 RViz 显示，可选写 CSV
        Node(
            package='auv_end2end',
            executable='trajectory_recorder',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'csv_path': LaunchConfiguration('record_csv'),
            }]
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])
