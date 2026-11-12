from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mass', default_value='1.6', description='起飞质量（kg）'),
        DeclareLaunchArgument('altitude', default_value='0.0', description='海拔（m）'),
        Node(
            package='uav_hover_throttle',
            executable='main_node',
            name='uav_hover_throttle',
            output='screen',
            parameters=[{
                'mass': ParameterValue(LaunchConfiguration('mass'), value_type=float),
                'altitude': ParameterValue(
                    LaunchConfiguration('altitude'), value_type=float),
            }],
        ),
    ])