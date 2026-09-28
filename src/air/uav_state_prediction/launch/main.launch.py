from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config=str(Path(get_package_share_directory('uav_state_prediction'))/'rviz/prediction.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('model',description='Absolute path to locally trained checkpoint'),
        DeclareLaunchArgument('csv',default_value=''),
        DeclareLaunchArgument('host',default_value='192.168.239.1'),
        DeclareLaunchArgument('vehicle',default_value='PredictionDrone'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('plot',default_value='true'),
        DeclareLaunchArgument('error_log',default_value=''),
        Node(package='uav_state_prediction',executable='state_predictor',output='screen',
            arguments=['--model',LaunchConfiguration('model'),'--error-log',LaunchConfiguration('error_log')]),
        Node(package='uav_state_prediction',executable='state_source',output='screen',
            arguments=['--host',LaunchConfiguration('host'),'--vehicle',LaunchConfiguration('vehicle'),'--csv',LaunchConfiguration('csv')]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',config],condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='uav_state_prediction',executable='topic_plot',output='screen',condition=IfCondition(LaunchConfiguration('plot')))
    ])
