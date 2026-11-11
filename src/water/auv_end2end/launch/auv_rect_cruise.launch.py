import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('auv_end2end'),
        'worlds',
        'underwater.world'
    )

    return LaunchDescription([
        # 巡航参数，可在命令行覆盖：
        # ros2 launch auv_end2end auv_rect_cruise.launch.py forward_speed:=3.0
        DeclareLaunchArgument('forward_speed', default_value='2.5'),
        DeclareLaunchArgument('side_speed', default_value='2.0'),
        DeclareLaunchArgument('forward_time', default_value='10.0'),
        DeclareLaunchArgument('side_time', default_value='20.0'),
        # 启动Gazebo仿真
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen',
            name='gazebo_sim'
        ),
        # 启动矩形巡航节点
        Node(
            package='auv_end2end',
            executable='auv_rect_cruise',
            name='auv_rect_cruise_node',
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
        )
    ])
