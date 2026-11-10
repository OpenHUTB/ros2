"""SLAM + 规划 + 控制全链路：仿真 / SLAM / 导航三个节点。

    ros2 launch ugv_visual_slam_nav main.launch.py
    ros2 launch ugv_visual_slam_nav main.launch.py scene:=cluttered seed:=1

rviz2 不在这里启动（避免给 headless 环境引入额外依赖）。
需要可视化时另开终端 `rviz2`，手动添加 Map(/ugv/map)、Path(/ugv/path)、
PoseArray(/ugv/landmarks) 三个显示项即可，话题都是标准类型。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    seed = LaunchConfiguration('seed')

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='corridor',
                              description='corridor | cluttered | dynamic | night'),
        DeclareLaunchArgument('seed', default_value='0'),
        LogInfo(msg='ugv_visual_slam_nav: 启动 sim + slam + nav（10 Hz 闭环）'),

        Node(package='ugv_visual_slam_nav', executable='sim',
             name='ugv_sim', output='screen',
             parameters=[{'scene': scene, 'seed': seed, 'rate_hz': 10.0}]),

        Node(package='ugv_visual_slam_nav', executable='slam',
             name='ugv_slam', output='screen',
             parameters=[{'map_rate_hz': 2.0, 'occ_threshold': 0.62,
                          'start_x': 2.0, 'start_y': 2.0, 'start_yaw': 0.0}]),

        Node(package='ugv_visual_slam_nav', executable='nav',
             name='ugv_nav', output='screen',
             parameters=[{'goal_x': 17.6, 'goal_y': 11.6,
                          'control_hz': 10.0, 'spin_time': 1.6,
                          'max_speed': 0.6}]),
    ])
