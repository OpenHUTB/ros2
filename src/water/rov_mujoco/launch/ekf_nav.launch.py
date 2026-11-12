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
    max_dvl_jump = LaunchConfiguration('max_dvl_jump')
    max_dvl_speed = LaunchConfiguration('max_dvl_speed')
    max_dvl_gap = LaunchConfiguration('max_dvl_gap')
    dvl_jump_reset_count = LaunchConfiguration('dvl_jump_reset_count')
    dvl_outlier_ratio = LaunchConfiguration('dvl_outlier_ratio')
    dvl_outlier_scale = LaunchConfiguration('dvl_outlier_scale')

    return LaunchDescription([
        DeclareLaunchArgument('dvl_dropout', default_value='0.10',
                              description='DVL 丢包率 (0-1)'),
        DeclareLaunchArgument('use_ocean_current', default_value='true',
                              description='是否启用三层洋流扰动'),
        DeclareLaunchArgument('max_dvl_jump', default_value='0.0',
                              description='相邻 DVL 速度跳变门限 (m/s, 0 = 关闭)'),
        DeclareLaunchArgument('max_dvl_speed', default_value='0.0',
                              description='DVL 速度模长门限 (m/s, 0 = 关闭)'),
        DeclareLaunchArgument('max_dvl_gap', default_value='2.0',
                              description='超过该间隔后重新捕获 DVL 跳变基准 (s)'),
        DeclareLaunchArgument('dvl_jump_reset_count', default_value='10',
                              description='连续跳变拒绝后重置基准的次数，0 = 关闭'),
        DeclareLaunchArgument('dvl_outlier_ratio', default_value='0.0',
                              description='仿真 DVL 野值注入概率 (0-1)'),
        DeclareLaunchArgument('dvl_outlier_scale', default_value='2.0',
                              description='仿真 DVL 野值基础幅值 (m/s)'),

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
                'dvl_outlier_ratio': ParameterValue(
                    dvl_outlier_ratio, value_type=float),
                'dvl_outlier_scale': ParameterValue(
                    dvl_outlier_scale, value_type=float),
            }],
        ),
        Node(
            package='rov_mujoco',
            executable='ekf_nav_node',
            name='ekf_nav_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'max_dvl_jump': ParameterValue(max_dvl_jump, value_type=float),
                'max_dvl_speed': ParameterValue(max_dvl_speed, value_type=float),
                'max_dvl_gap': ParameterValue(max_dvl_gap, value_type=float),
                'dvl_jump_reset_count': ParameterValue(
                    dvl_jump_reset_count, value_type=int),
            }],
        ),
    ])
