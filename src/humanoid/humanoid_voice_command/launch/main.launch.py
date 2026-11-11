"""语音指令控制 launch 文件。

用法::

    # 单条口令（识别完就退出仿真仍在跑）
    ros2 launch humanoid_voice_command main.launch.py wav:=<包路径>/audio/forward_1.wav

    # 目录轮询：按顺序播放全部口令，每 3 s 一条
    ros2 launch humanoid_voice_command main.launch.py audio_dir:=<包路径>/audio

    # 无图形界面（服务器 / 容器）
    ros2 launch humanoid_voice_command main.launch.py visualize:=false

启动节点：
    voice_node     口令音频 -> /voice_text、/voice_intent
    command_node   意图 -> /cmd_vel、/humanoid/gesture
    mujoco_sim_node  MuJoCo 仿真（复用 humanoid_keyboard_control 包）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_voice_command')
    default_audio = os.path.join(pkg_share, 'audio')

    args = [
        DeclareLaunchArgument('wav', default_value='',
                              description='单条口令音频路径（与 audio_dir 二选一）'),
        DeclareLaunchArgument('audio_dir', default_value=default_audio,
                              description='口令音频目录，按文件名顺序轮询'),
        DeclareLaunchArgument('interval', default_value='3.0',
                              description='目录轮询时两条口令的间隔 (s)'),
        DeclareLaunchArgument('loop', default_value='true',
                              description='目录轮询是否循环'),
        DeclareLaunchArgument('model', default_value='tiny',
                              description='Whisper 模型规格'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='本地 Whisper 权重路径（空则用缓存或自动下载）'),
        DeclareLaunchArgument('language', default_value='zh',
                              description='识别语言（zh / en / auto）'),
        DeclareLaunchArgument('hold_time', default_value='3.0',
                              description='单条运动指令保持时长 (s)'),
        DeclareLaunchArgument('start_sim', default_value='true',
                              description='是否同时启动 MuJoCo 仿真节点'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='是否打开 MuJoCo 可视化窗口'),
    ]

    voice_node = Node(
        package='humanoid_voice_command',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'wav': LaunchConfiguration('wav'),
            'audio_dir': LaunchConfiguration('audio_dir'),
            'interval': LaunchConfiguration('interval'),
            'loop': LaunchConfiguration('loop'),
            'model': LaunchConfiguration('model'),
            'model_path': LaunchConfiguration('model_path'),
            'language': LaunchConfiguration('language'),
        }],
    )

    command_node = Node(
        package='humanoid_voice_command',
        executable='command_node',
        name='command_node',
        output='screen',
        parameters=[{
            'hold_time': LaunchConfiguration('hold_time'),
        }],
    )

    sim_node = Node(
        package='humanoid_keyboard_control',
        executable='mujoco_sim_node',
        name='mujoco_sim_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_sim')),
        parameters=[{
            'cmd_timeout': 0.5,
            'visualize': LaunchConfiguration('visualize'),
        }],
    )

    return LaunchDescription(args + [voice_node, command_node, sim_node])
