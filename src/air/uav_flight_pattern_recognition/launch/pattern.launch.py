from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    replay = Node(package='uav_flight_pattern_recognition', executable='pattern_replay',
                  parameters=[{'files': LaunchConfiguration('files'), 'speed': 1.}], output='screen')
    recognizer = Node(package='uav_flight_pattern_recognition', executable='pattern_recognizer',
                      parameters=[{'model': LaunchConfiguration('model'), 'log': LaunchConfiguration('log')}], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('files'), DeclareLaunchArgument('model'), DeclareLaunchArgument('log', default_value=''),
        recognizer, replay,
        RegisterEventHandler(OnProcessExit(target_action=replay, on_exit=[EmitEvent(event=Shutdown(reason='Replay ended'))]))
    ])
