#!/usr/bin/env python3
"""launch 启动 RL 算法 / 网络结构对比训练

用法（参数可覆盖）：
  ros2 launch rl_algorithm_compare main.launch.py
  ros2 launch rl_algorithm_compare main.launch.py algo:=dqn layers:='64,64' tag:=dqn_mlp64
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    algo = LaunchConfiguration('algo')
    layers = LaunchConfiguration('layers')
    tag = LaunchConfiguration('tag')
    episodes = LaunchConfiguration('episodes')

    return LaunchDescription([
        DeclareLaunchArgument('algo', default_value='dqn',
                              description='算法: dqn/reinforce'),
        DeclareLaunchArgument('layers', default_value='16',
                              description='MLP 隐藏层，逗号分隔'),
        DeclareLaunchArgument('tag', default_value='dqn_mlp1',
                              description='结果文件名标签'),
        DeclareLaunchArgument('episodes', default_value='300',
                              description='训练回合数'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rl_algorithm_compare', 'train',
                 '--algo', algo, '--layers', layers, '--tag', tag,
                 '--episodes', episodes],
            output='screen'),
    ])
