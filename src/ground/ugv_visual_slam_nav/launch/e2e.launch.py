"""端到端策略：仿真节点 + CNN 推理节点，图像直接进、速度指令直接出。

    ros2 launch ugv_visual_slam_nav e2e.launch.py
    ros2 launch ugv_visual_slam_nav e2e.launch.py scene:=night seed:=2
    ros2 launch ugv_visual_slam_nav e2e.launch.py goal_x:=17.6 goal_y:=11.6

权重默认取 share/ugv_visual_slam_nav/models/e2e.npz（由 `python -m
ugv_visual_slam_nav.main --stage train` 生成），可用 model_path 覆盖。

注意 `max_speed` 默认压到 0.42 m/s：训练标签是按 0.6 m/s 归一化的，推理时
留 30% 的减速余量，用来抵消端到端策略没有显式避障层这件事。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    seed = LaunchConfiguration('seed')
    model = LaunchConfiguration('model_path')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='corridor'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('goal_x', default_value='17.6'),
        DeclareLaunchArgument('goal_y', default_value='11.6'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='留空则用 share 目录下的 models/e2e.npz'),
        LogInfo(msg='端到端闭环：图像 + 目标方位 -> CNN -> /ugv/cmd_vel'),

        Node(package='ugv_visual_slam_nav', executable='sim',
             name='ugv_sim', output='screen',
             parameters=[{'scene': scene, 'seed': seed, 'rate_hz': 10.0}]),

        Node(package='ugv_visual_slam_nav', executable='e2e',
             name='ugv_e2e', output='screen',
             parameters=[{'model_path': model, 'max_speed': 0.42,
                          'goal_x': goal_x, 'goal_y': goal_y}]),
    ])
