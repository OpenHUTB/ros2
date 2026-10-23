#!/usr/bin/env python3
"""launch 启动 DQN 改进算法（Double / Dueling）训练

用法（参数可覆盖）：
  ros2 launch rl_dqn_improvement main.launch.py
  ros2 launch rl_dqn_improvement main.launch.py variant:=double tag:=double_mlp1 episodes:=300
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    variant = LaunchConfiguration('variant')
    layers = LaunchConfiguration('layers')
    tag = LaunchConfiguration('tag')
    episodes = LaunchConfiguration('episodes')

    return LaunchDescription([
        DeclareLaunchArgument('variant', default_value='dqn',
                              description='DQN变体: dqn/double/dueling'),
        DeclareLaunchArgument('layers', default_value='16',
                              description='MLP 隐藏层，逗号分隔'),
        DeclareLaunchArgument('tag', default_value='dqn_mlp1',
                              description='结果文件名标签'),
        DeclareLaunchArgument('episodes', default_value='300',
                              description='训练回合数'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rl_dqn_improvement', 'train',
                 '--variant', variant, '--layers', layers,
                 '--tag', tag, '--episodes', episodes],
            output='screen'),
    ])
