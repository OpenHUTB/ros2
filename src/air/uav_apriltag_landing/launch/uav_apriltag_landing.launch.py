from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_apriltag_landing',
            executable='apriltag_landing_node',
            name='apriltag_landing',
            output='screen',
            parameters=[{
                'hover_height': 5.0,
                'align_pixel_tol': 15.0,
                'descend_step': 0.3,
                'landed_height': 0.1,
                'cam_fx': 500.0,
                'cam_fy': 500.0,
                'cam_cx': 320.0,
                'cam_cy': 240.0,
                'tag_family': 'tag36h11',
                'tag_size': 0.16,
                'target_tag_id': -1,
                'flip_x': False,
                'flip_y': False,
                'img_timeout': 1.0,
                'height_timeout': 2.0,
            }],
        ),
    ])
