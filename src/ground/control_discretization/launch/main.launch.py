from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='tustin',
                              description='forward/backward/tustin/zoh'),
        DeclareLaunchArgument('ts', default_value='0.05', description='采样周期（秒）'),
        Node(
            package='control_discretization',
            executable='main_node',
            name='control_discretization',
            output='screen',
            parameters=[{
                'method': ParameterValue(LaunchConfiguration('method'), value_type=str),
                'ts': ParameterValue(LaunchConfiguration('ts'), value_type=float),
            }],
        ),
    ])