"""训练启动：ros2 launch carla_rl_driving carla_rl_train.launch.py

训练需要先把 CARLA 模拟器跑起来（见文档"仿真步骤"），再用本 launch 拉起训练节点。
可通过 launch 参数覆盖算法/结构/场景，例如：
    ros2 launch carla_rl_driving carla_rl_train.launch.py algo:=sac arch:=cnn_lstm
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('algo', default_value='ppo',
                              description='RL 算法：ppo | sac'),
        DeclareLaunchArgument('arch', default_value='cnn',
                              description='网络结构：cnn | cnn_lstm'),
        DeclareLaunchArgument('scenario', default_value='town01',
                              description='测试场景：town01 | town02 | town04'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='2000'),

        Node(
            package='carla_rl_driving',
            executable='train',
            name='carla_rl_train',
            output='screen',
            arguments=[
                '--algo', LaunchConfiguration('algo'),
                '--arch', LaunchConfiguration('arch'),
                '--scenario', LaunchConfiguration('scenario'),
                '--host', LaunchConfiguration('host'),
                '--port', LaunchConfiguration('port'),
            ],
        ),
    ])
