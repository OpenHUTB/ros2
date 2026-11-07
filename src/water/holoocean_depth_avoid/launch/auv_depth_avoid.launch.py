from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='holoocean_depth_avoid',
            executable='auv_depth_avoid',
            name='auv_depth_avoid_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # 按自己的 HoloOcean2 深度相机话题名修改，
                # HoloOcean2 默认形如: /holoocean/auv0/DepthCamera
                'depth_topic': '/holoocean/auv0/DepthCamera',
                'cmd_topic': '/auv/cmd_vel',
                'danger_dist': 1.0,
                'warn_dist': 2.0,
                'cruise_speed': 0.8,
                'target_altitude': 1.5,
            }],
            remappings=[
                # 若速度话题在 HoloOcean2 里叫别的名字，在这里重映射
                ('/auv/cmd_vel', '/auv/cmd_vel'),
            ],
        )
    ])
