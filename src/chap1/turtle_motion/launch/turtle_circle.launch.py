from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    radius = LaunchConfiguration('radius')
    linear_speed = LaunchConfiguration('linear_speed')
    turtle = LaunchConfiguration('turtle')

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='2.0',
                              description='圆的半径 [m]'),
        DeclareLaunchArgument('linear_speed', default_value='2.0',
                              description='线速度 [m/s]'),
        DeclareLaunchArgument('turtle', default_value='turtle1',
                              description='海龟名字'),
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='turtle_motion', executable='main',
             name='turtle_circle_node', output='screen',
             parameters=[{
                 'radius': ParameterValue(radius, value_type=float),
                 'linear_speed': ParameterValue(linear_speed, value_type=float),
                 'turtle': ParameterValue(turtle, value_type=str),
             }]),
    ])
