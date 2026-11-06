from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    zmp_node = Node(
        package='humanoid_zmp_walking',
        executable='zmp_generator',
        name='zmp_generator_node',
        output='screen'
    )
    
    play_node = Node(
        package='humanoid_zmp_walking',
        executable='play',
        name='play_node',
        output='screen'
    )
    
    return LaunchDescription([
        zmp_node,
        TimerAction(period=2.0, actions=[play_node])
    ])
