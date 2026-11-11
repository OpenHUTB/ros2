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
        # 避障参数，可在命令行覆盖：
        # ros2 launch auv_end2end auv_sim.launch.py trigger_ratio:=0.3
        DeclareLaunchArgument('obstacle_thresh', default_value='22'),
        DeclareLaunchArgument('trigger_ratio', default_value='0.22'),
        DeclareLaunchArgument('recover_ratio', default_value='0.18'),
        DeclareLaunchArgument('forward_speed', default_value='4.5'),
        DeclareLaunchArgument('side_speed', default_value='2.8'),
        DeclareLaunchArgument('max_avoid_duration', default_value='3.0'),
        DeclareLaunchArgument('avoid_cooldown', default_value='2.0'),
        # 启动Gazebo仿真
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen',
            name='gazebo_sim'
        ),
        # 启动视觉避障节点（节点名沿用代码里的 auv_obstacle_avoid）
        Node(
            package='auv_end2end',
            executable='auv_obstacle_avoid',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'obstacle_thresh': ParameterValue(
                    LaunchConfiguration('obstacle_thresh'), value_type=int),
                'trigger_ratio': ParameterValue(
                    LaunchConfiguration('trigger_ratio'), value_type=float),
                'recover_ratio': ParameterValue(
                    LaunchConfiguration('recover_ratio'), value_type=float),
                'forward_speed': ParameterValue(
                    LaunchConfiguration('forward_speed'), value_type=float),
                'side_speed': ParameterValue(
                    LaunchConfiguration('side_speed'), value_type=float),
                'max_avoid_duration': ParameterValue(
                    LaunchConfiguration('max_avoid_duration'),
                    value_type=float),
                'avoid_cooldown': ParameterValue(
                    LaunchConfiguration('avoid_cooldown'), value_type=float),
            }]
        )
    ])
