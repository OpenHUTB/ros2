from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_gnss_integrity_monitor',
            executable='gnss_integrity_node',
            name='gnss_integrity_monitor',
            output='screen',
            parameters=[{
                'max_speed': 12.0,
                'max_vertical_speed': 6.0,
                'position_margin': 1.5,
                'accuracy_limit': 3.0,
                'min_fix_type': 2,
                'min_sats': 8,
                'stale_timeout': 1.5,
                'max_extrapolate_dt': 0.5,
                'suspect_limit': 2,
                'recovery_streak': 5,
                'reacquisition_radius': 3.0,
                'origin_lat': 0.0,
                'origin_lon': 0.0,
                'origin_alt': 0.0,
                'default_accuracy': 0.5,
                'frame_id': 'map',
            }],
        ),
    ])
