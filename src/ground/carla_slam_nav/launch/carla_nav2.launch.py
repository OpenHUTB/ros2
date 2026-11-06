"""导航启动：ros2 launch carla_slam_nav carla_nav2.launch.py

拉起 Nav2 导航栈（BT 导航器 + 全局规划器 + 局部控制器 + 代价地图），
订阅 SLAM 输出的 /map，接收 /initialpose 与 /move_base_simple/goal 后自主导航。
需先启动 carla_slam.launch.py（提供 map 与 TF）。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    params = os.path.join(
        get_package_share_directory('carla_slam_nav'), 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'bringup_launch.py')),
            launch_arguments={
                'map': '',                       # 用 SLAM 实时地图，不加载静态地图
                'use_sim_time': 'True',
                'params_file': params,
                'autostart': 'True',
            }.items(),
        ),
    ])
