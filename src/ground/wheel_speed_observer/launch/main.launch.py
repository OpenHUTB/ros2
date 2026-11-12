from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bandwidth_factor', default_value='1.0',
                              description='观测器带宽相对开环固有频率的倍数'),
        Node(
            package='wheel_speed_observer',
            executable='main_node',
            name='wheel_speed_observer',
            output='screen',
            parameters=[{
                'bandwidth_factor': ParameterValue(
                    LaunchConfiguration('bandwidth_factor'), value_type=float),
            }],
        ),
    ])