"""EKF 多源融合导航一键启动（仿真 + 传感器噪声 + 融合节点）"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('rov_mujoco')
    model = os.path.join(share, 'models', 'underwater_rov_with_arm.xml')
    config = os.path.join(share, 'models', 'config.json')

    dropout = LaunchConfiguration('dvl_dropout')
    current = LaunchConfiguration('use_ocean_current')

    return LaunchDescription([
        DeclareLaunchArgument('dvl_dropout', default_value='0.10',
                              description='DVL 丢包率 (0-1)'),
        DeclareLaunchArgument('use_ocean_current', default_value='true',
                              description='是否启用三层洋流扰动'),

        Node(
            package='rov_mujoco',
            executable='mujoco_sim_node',
            name='mujoco_sim_node',
            output='screen',
            parameters=[{
                'model_path': model,
                'config_path': config,
                'auto_trajectory_mode': True,
                'controller_type': 'pid',
                'use_ocean_current': ParameterValue(current, value_type=bool),
            }],
        ),
        Node(
            package='rov_mujoco',
            executable='sensor_sim_node',
            name='sensor_sim_node',
            output='screen',
            parameters=[{
                'dvl_dropout': ParameterValue(dropout, value_type=float),
            }],
        ),
        Node(
            package='rov_mujoco',
            executable='ekf_nav_node',
            name='ekf_nav_node',
            output='screen',
        ),
    ])
