"""启动禁飞区检测节点，可通过 launch 参数指定期望 frame_id."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构建 geofence_node 的启动描述."""
    expected_frame_id = LaunchConfiguration('expected_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'expected_frame_id', default_value='map',
            description='期望的坐标系 frame_id，用于一致性校验'),
        Node(
            package='geofence_demo',
            executable='geofence_node',
            name='geofence_node',
            output='screen',
            parameters=[{
                'expected_frame_id': expected_frame_id,
            }],
        ),
    ])
