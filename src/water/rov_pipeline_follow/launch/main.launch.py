"""SLAM + 感知 + 控制全链路：仿真 / SLAM / 控制三个节点。

    ros2 launch rov_pipeline_follow main.launch.py
    ros2 launch rov_pipeline_follow main.launch.py scene:=turbid seed:=1
    ros2 launch rov_pipeline_follow main.launch.py use_cnn:=true

rviz2 不在这里启动（避免给 headless 环境引入额外依赖）。
需要可视化时另开终端 `rviz2`，手动添加 Map(/rov/map)、Path(/rov/path)、
PoseArray(/rov/landmarks) 三个显示项即可，话题都是标准类型。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    seed = LaunchConfiguration('seed')
    use_cnn = LaunchConfiguration('use_cnn')

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='straight',
                              description='straight | curve | current | turbid'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('use_cnn', default_value='false',
                              description='true 时控制节点用 CNN 感知'),
        LogInfo(msg='rov_pipeline_follow: 启动 sim + slam + control（10 Hz 闭环）'),

        Node(package='rov_pipeline_follow', executable='sim',
             name='rov_sim', output='screen',
             parameters=[{'scene': scene, 'seed': seed, 'rate_hz': 10.0}]),

        Node(package='rov_pipeline_follow', executable='slam',
             name='rov_slam', output='screen',
             parameters=[{'scene': scene, 'seed': seed,
                          'map_rate_hz': 2.0, 'occ_threshold': 0.62}]),

        Node(package='rov_pipeline_follow', executable='control',
             name='rov_control', output='screen',
             parameters=[{'scene': scene, 'seed': seed,
                          'use_cnn': use_cnn, 'control_hz': 10.0}]),
    ])
