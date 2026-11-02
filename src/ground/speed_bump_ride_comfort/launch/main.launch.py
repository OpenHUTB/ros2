from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="speed_bump_ride_comfort",
            executable="main",
            name="speed_bump_ride_comfort",
            output="screen",
            parameters=[{
                "bump_height_m": 0.06,
                "bump_width_m": 1.2,
                "bump_kind": "half_sine",
                "comfort_limit_mps2": 3.0,
                "publish_period_s": 0.2,
                "simulation_dt_s": 0.001,
            }],
        )
    ])
