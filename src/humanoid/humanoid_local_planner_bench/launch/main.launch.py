"""人形局部避障对比 Launch。

用法：
    ros2 launch humanoid_local_planner_bench main.launch.py
    ros2 launch humanoid_local_planner_bench main.launch.py planner:=apf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'humanoid_local_planner_bench'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(share, 'config', 'params.yaml')
    default_scene = os.path.join(share, 'models', 'corridor_scene.xml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ROS 参数文件路径'),
        DeclareLaunchArgument('scene', default_value=default_scene,
                              description='MuJoCo 场景 XML 路径'),
        DeclareLaunchArgument('planner', default_value='dwa',
                              description='局部规划器：dwa 或 apf'),
        Node(
            package=PACKAGE_NAME,
            executable='planner_bench_node',
            name='planner_bench_node',   # 必须与 config/params.yaml 的节点键一致，否则参数不生效
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {'scene': LaunchConfiguration('scene')},
                {'planner': LaunchConfiguration('planner')},
            ],
        ),
    ])
