from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros2_opencv_tracking',
            executable='image_pub',
            name='image_pub',
            output='screen',
            parameters=[{
                'image_width': 640,
                'image_height': 480,
                'radius_px': 35,
                'rate_hz': 10.0,
            }],
        ),
        Node(
            package='px4_ros2_opencv_tracking',
            executable='color_tracker',
            name='color_tracker',
            output='screen',
            parameters=[{
                'min_area': 300.0,
                'publish_debug_image': True,
            }],
        ),
        Node(
            package='px4_ros2_opencv_tracking',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
            parameters=[{
                'takeoff_alt': -2.0,
                'desired_area': 3750.0,
                'max_v': 1.0,
                'k_y': 0.12,
                'k_d': 0.02,
                'k_alt': 0.8,
                'min_area': 300.0,
                'target_timeout': 3.0,
                'deadband_px': 15.0,
                'deadband_area_ratio': 0.05,
                'state_timeout': 10.0,
            }],
        ),
    ])
