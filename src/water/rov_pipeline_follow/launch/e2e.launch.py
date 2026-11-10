"""端到端策略：仿真节点 + CNN 推理节点，图像直接进、速度指令直接出。

    ros2 launch rov_pipeline_follow e2e.launch.py
    ros2 launch rov_pipeline_follow e2e.launch.py scene:=turbid seed:=2

权重默认取 share/rov_pipeline_follow/models/e2e.npz（由 `python -m
rov_pipeline_follow.main --stage train` 生成），可用 model_path 覆盖。

注意 `max_speed` 默认压到 0.45 m/s：训练标签是按 0.55 m/s 归一化的，
推理时留 20% 的减速余量，用来抵消端到端策略没有显式避障层这件事。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    seed = LaunchConfiguration('seed')
    model = LaunchConfiguration('model_path')

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='straight'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='留空则用 share 目录下的 models/e2e.npz'),
        LogInfo(msg='端到端闭环：图像 + 深度 + 目标方位 -> CNN -> /rov/cmd_vel'),

        Node(package='rov_pipeline_follow', executable='sim',
             name='rov_sim', output='screen',
             parameters=[{'scene': scene, 'seed': seed, 'rate_hz': 10.0}]),

        Node(package='rov_pipeline_follow', executable='e2e',
             name='rov_e2e', output='screen',
             parameters=[{'model_path': model, 'max_speed': 0.45,
                          'scene': scene, 'seed': seed}]),
    ])
