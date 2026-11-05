from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('files'),DeclareLaunchArgument('model'),
        DeclareLaunchArgument('log',default_value='results/ros_predictions.jsonl'),
        Node(package='landing_assessment',executable='landing_assessment_node',parameters=[{'model':LaunchConfiguration('model'),'log':LaunchConfiguration('log')}],output='screen'),
        Node(package='landing_assessment',executable='landing_replay',parameters=[{'files':LaunchConfiguration('files')}],output='screen')])
