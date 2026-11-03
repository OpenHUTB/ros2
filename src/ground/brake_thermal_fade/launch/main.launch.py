from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="brake_thermal_fade",
            executable="main",
            name="brake_thermal_fade",
            output="screen",
            parameters=[{
                "stop_speed_kmh": 100.0,
                "ambient_c": 25.0,
                "fade_start_c": 250.0,
                "fade_end_c": 650.0,
                "mu_cold": 0.40,
                "mu_hot": 0.22,
            }],
        )
    ])
