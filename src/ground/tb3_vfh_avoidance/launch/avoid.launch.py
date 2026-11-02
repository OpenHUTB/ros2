"""launch 文件：加载 config/params.yaml 启动避障节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('tb3_vfh_avoidance'),
                          'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='tb3_vfh_avoidance',
            executable='avoid_node',
            name='tb3_vfh_avoidance',
            output='screen',
            parameters=[params],
        ),
    ])
