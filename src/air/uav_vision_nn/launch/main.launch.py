"""推理 launch：AirSim 桥 + 视觉端到端推理节点。

用法：
    ros2 launch uav_vision_nn main.launch.py
    ros2 launch uav_vision_nn main.launch.py model_path:=/path/to/model.pth
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 遥测话题统一挂到 /uav 命名空间下，节点内部仍用 ~/image 这类相对名，
# 靠 remap 把它们接到同一个图上，保证 launch 一起启动即可闭环。
UAV_TOPIC_REMAPS = [
    ('~/image', '/uav/image'),
    ('~/pose', '/uav/pose'),
    ('~/depth', '/uav/depth'),
    ('~/cmd_vel', '/uav/cmd_vel'),
]

UAV_SERVICE_REMAPS = [
    ('~/takeoff', '/uav/takeoff'),
    ('~/land', '/uav/land'),
]


def generate_launch_description():
    share_dir = get_package_share_directory('uav_vision_nn')
    params_file = os.path.join(share_dir, 'config', 'params.yaml')

    default_model = os.path.join(
        os.path.expanduser('~'), '.ros', 'uav_vision_nn', 'model', 'model.pth')

    airsim_ip = LaunchConfiguration('airsim_ip')
    airsim_port = LaunchConfiguration('airsim_port')
    vehicle_name = LaunchConfiguration('vehicle_name')
    model_path = LaunchConfiguration('model_path')

    arguments = [
        DeclareLaunchArgument(
            'airsim_ip', default_value='10.0.2.2',
            description='AirSim 所在主机地址，VirtualBox NAT 下宿主机为 10.0.2.2'),
        DeclareLaunchArgument(
            'airsim_port', default_value='41451',
            description='AirSim RPC 端口（settings.json 的 ApiServerPort）'),
        DeclareLaunchArgument(
            'vehicle_name', default_value='',
            description='AirSim 载具名，默认载具留空'),
        DeclareLaunchArgument(
            'model_path', default_value=default_model,
            description='训练得到的权重文件绝对路径'),
    ]

    airsim_source = Node(
        package='uav_vision_nn',
        executable='airsim_source',
        name='airsim_source',
        output='screen',
        parameters=[params_file, {
            'airsim_ip': ParameterValue(airsim_ip, value_type=str),
            'airsim_port': ParameterValue(airsim_port, value_type=int),
            'vehicle_name': ParameterValue(vehicle_name, value_type=str),
        }],
        remappings=UAV_TOPIC_REMAPS + UAV_SERVICE_REMAPS,
    )

    inference = Node(
        package='uav_vision_nn',
        executable='main',
        name='uav_vision_node',
        output='screen',
        parameters=[params_file, {'model_path': model_path}],
        remappings=UAV_TOPIC_REMAPS + [('~/infer_ms', '/uav/infer_ms')],
    )

    return LaunchDescription(arguments + [airsim_source, inference])
