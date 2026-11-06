"""人形上下台阶 Launch。

用法：
    ros2 launch humanoid_stair_climb main.launch.py
    ros2 launch humanoid_stair_climb main.launch.py cycle_time:=1.6
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'humanoid_stair_climb'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(share, 'config', 'params.yaml')
    default_scene = os.path.join(share, 'models', 'stair_scene.xml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ROS 参数文件路径'),
        DeclareLaunchArgument('scene', default_value=default_scene,
                              description='MuJoCo 场景 XML 路径'),
        DeclareLaunchArgument('cycle_time', default_value='1.4',
                              description='一个完整步态周期，秒'),
        Node(
            package=PACKAGE_NAME,
            executable='stair_climb_node',
            name='stair_climb_node',   # 必须与 config/params.yaml 的节点键一致，否则参数不生效
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {'scene': LaunchConfiguration('scene')},
                {'cycle_time': LaunchConfiguration('cycle_time')},
            ],
        ),
    ])
