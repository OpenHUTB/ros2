import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('water_rov_path_planning')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # 两个节点都只加载同一份 params.yaml，避免 launch 与 yaml 重复定义参数
    planning_node = Node(
        package='water_rov_path_planning',
        executable='planning_node',
        name='rov_planning_node',
        output='screen',
        parameters=[params_file],
    )

    demo_node = Node(
        package='water_rov_path_planning',
        executable='demo_node',
        name='rov_demo_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        planning_node,
        demo_node,
    ])
