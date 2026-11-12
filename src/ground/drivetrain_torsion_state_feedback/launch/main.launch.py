from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('zeta', default_value='0.7',
                              description='闭环目标阻尼比'),
        DeclareLaunchArgument('feedback_enabled', default_value='true',
                              description='false 时退化为开环对比'),
        Node(
            package='drivetrain_torsion_state_feedback',
            executable='main_node',
            name='drivetrain_torsion_state_feedback',
            output='screen',
            parameters=[{
                'zeta': ParameterValue(LaunchConfiguration('zeta'), value_type=float),
                'feedback_enabled': ParameterValue(
                    LaunchConfiguration('feedback_enabled'), value_type=bool),
            }],
        ),
    ])