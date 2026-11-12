from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    side = LaunchConfiguration('side_length')
    linear = LaunchConfiguration('linear_speed')
    angular = LaunchConfiguration('angular_speed')
    turtle = LaunchConfiguration('turtle')

    return LaunchDescription([
        DeclareLaunchArgument('side_length', default_value='1.0',
                              description='五角星边长 [m]'),
        DeclareLaunchArgument('linear_speed', default_value='1.0',
                              description='直行线速度 [m/s]'),
        DeclareLaunchArgument('angular_speed', default_value='1.0',
                              description='原地转角速度 [rad/s]'),
        DeclareLaunchArgument('turtle', default_value='turtle1'),
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='turtle_star', executable='main',
             name='turtle_star_node', output='screen',
             parameters=[{
                 'side_length': ParameterValue(side, value_type=float),
                 'linear_speed': ParameterValue(linear, value_type=float),
                 'angular_speed': ParameterValue(angular, value_type=float),
                 'turtle': ParameterValue(turtle, value_type=str),
             }]),
    ])
