from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_battery_rth_failsafe',
            executable='battery_rth_node',
            name='battery_rth_failsafe',
            output='screen',
            parameters=[{
                'warn_percent': 25.0,
                'rth_percent': 15.0,
                'emergency_percent': 8.0,
                'home_x': 0.0,
                'home_y': 0.0,
                'home_z': 0.0,
            }],
        ),
    ])
