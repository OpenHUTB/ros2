"""启动一次地面坡度扫描。

用法:
    ros2 launch ground_slope_estimation main.launch.py
    ros2 launch ground_slope_estimation main.launch.py slope_deg:=15.0 noise_std:=0.005
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    slope_deg = LaunchConfiguration('slope_deg')
    noise_std = LaunchConfiguration('noise_std')
    out_dir = LaunchConfiguration('out_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slope_deg', default_value='10.0',
            description='地面真实倾角（度）'),
        DeclareLaunchArgument(
            'noise_std', default_value='0.0',
            description='深度噪声标准差（米）'),
        DeclareLaunchArgument(
            'out_dir', default_value='results',
            description='CSV 输出目录'),
        Node(
            package='ground_slope_estimation',
            executable='slope_sim',
            name='slope_sim',
            output='screen',
            parameters=[{
                'slope_deg': ParameterValue(slope_deg, value_type=float),
                'noise_std': ParameterValue(noise_std, value_type=float),
                'out_dir': ParameterValue(out_dir, value_type=str),
            }],
        ),
    ])
