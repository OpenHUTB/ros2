from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    horizon = LaunchConfiguration('horizon_s')
    jackknife = LaunchConfiguration('jackknife_angle_deg')

    return LaunchDescription([
        DeclareLaunchArgument('horizon_s', default_value='5.0',
                              description='折叠预测时域 [s]'),
        DeclareLaunchArgument('jackknife_angle_deg', default_value='75.0',
                              description='折叠角阈值 [deg]'),
        Node(package='trailer_jackknife_guard', executable='main',
             name='trailer_jackknife_guard', output='screen',
             arguments=['--node'],
             parameters=[{
                 'horizon_s': ParameterValue(horizon, value_type=float),
                 'jackknife_angle_deg': ParameterValue(jackknife,
                                                       value_type=float),
             }]),
    ])
