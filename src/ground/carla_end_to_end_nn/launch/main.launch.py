"""CARLA 端到端神经网络（图像 → 控制）—— ROS 2 Humble Launch 文件。

使用方式：
    ros2 launch carla_end_to_end_nn main.launch.py
    ros2 launch carla_end_to_end_nn main.launch.py host:=192.168.8.1 model_path:=models/cnn.json

说明：
    前视相机图像直接送入端到端 CNN 输出转向，中间不含人工分层的感知/规划环节。
    CARLA 服务端启动、宿主机 IP 与端口 2000 的查看等通用步骤见
    https://openhutb.github.io/ros2/set_up_and_connect_to_carla/ ，本模块不重复描述。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('carla_end_to_end_nn')
    default_config = os.path.join(pkg_share, 'config', 'sim_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'host', default_value='127.0.0.1',
            description='CARLA 服务端地址（虚拟机运行客户端时填宿主机 IP）'),
        DeclareLaunchArgument('port', default_value='2000', description='CARLA RPC 端口'),
        DeclareLaunchArgument('town', default_value='Town05', description='CARLA 地图名'),
        DeclareLaunchArgument(
            'model_path', default_value='models/cnn.json',
            description='端到端 CNN 模型路径（由 main.py --mode train 生成）'),
        DeclareLaunchArgument(
            'backend', default_value='numpy',
            description='numpy=纯numpy SimpleCNN（默认，无需 TensorFlow）；tf=tf.keras'),
        DeclareLaunchArgument(
            'config', default_value=default_config, description='仿真参数 YAML 路径'),

        Node(
            package='carla_end_to_end_nn',
            executable='end_to_end_node',
            name='carla_end_to_end_node',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'town': LaunchConfiguration('town'),
                'model_path': LaunchConfiguration('model_path'),
                'backend': LaunchConfiguration('backend'),
            }],
        ),
    ])
