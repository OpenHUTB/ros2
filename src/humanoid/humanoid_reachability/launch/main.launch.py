import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('humanoid_reachability')
    params_file = os.path.join(package_share, 'config', 'params.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'reachability.rviz')

    declare_samples = DeclareLaunchArgument(
        'samples_per_arm', default_value='200000',
        description='每臂随机采样次数')
    declare_resolution = DeclareLaunchArgument(
        'voxel_resolution', default_value='0.03',
        description='体素边长（m）')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='是否同时启动 RViz2')

    reachability_node = Node(
        package='humanoid_reachability',
        executable='humanoid_reachability_node',
        name='humanoid_reachability',
        parameters=[params_file, {
            'samples_per_arm': LaunchConfiguration('samples_per_arm'),
            'voxel_resolution': LaunchConfiguration('voxel_resolution'),
        }],
        output='screen',
    )

    arm_demo_node = Node(
        package='humanoid_reachability',
        executable='arm_demo_node',
        name='arm_demo_node',
        parameters=[params_file],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        declare_samples,
        declare_resolution,
        declare_use_rviz,
        reachability_node,
        arm_demo_node,
        rviz_node,
    ])
