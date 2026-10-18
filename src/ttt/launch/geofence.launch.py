from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geofence_detector',
            executable='geofence_node',
            name='geofence_node',
            output='screen', # 将终端输出打印到屏幕上
            # 如果以后想加参数，可以在这里配置 parameters
            # parameters=[{'margin_distance': 5.0}]
        )
    ])
