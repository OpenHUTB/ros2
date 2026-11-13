from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_mag_interference_monitor',
            executable='mag_interference_node',
            name='mag_interference_monitor',
            output='screen',
            parameters=[{
                'bias_x': 0.0,
                'bias_y': 0.0,
                'expected_radius': 0.0,
                'declination_deg': 0.0,
                'min_ground_speed': 2.0,
                'max_heading_error_deg': 15.0,
                'critical_heading_error_deg': 30.0,
                'norm_tolerance_pct': 15.0,
                'degraded_limit': 3,
                'unreliable_limit': 8,
                'recovery_streak': 5,
                'stale_timeout': 1.0,
                'frame_id': 'base_link',
            }],
        ),
    ])
