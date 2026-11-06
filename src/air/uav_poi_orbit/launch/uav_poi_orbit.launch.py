from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_poi_orbit',
            executable='poi_orbit_node',
            name='uav_poi_orbit',
            output='screen',
            parameters=[{
                'poi_x': 0.0,
                'poi_y': 0.0,
                'orbit_radius': 5.0,
                'orbit_height': 8.0,
                'omega_dps': 15.0,
                'spiral': False,
                'spiral_height_delta': 2.0,
                'waypoint_interval_s': 0.2,
                'min_height': 0.5,
            }],
        ),
    ])
