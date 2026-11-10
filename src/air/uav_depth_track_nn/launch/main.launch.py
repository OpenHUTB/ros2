"""推理 launch：AirSim 桥 + 深度跟踪推理节点。

用法：
    ros2 launch uav_depth_track_nn main.launch.py \\
        airsim_port:=8080 vehicle_name:=drone1 \\
        model_path:=$HOME/.ros/uav_depth_track_nn/model/model.pth

注意：launch 不转发终端 stdin，起飞请另开终端调用
    ros2 service call /uav/takeoff std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    share_dir = get_package_share_directory('uav_depth_track_nn')
    params_file = os.path.join(share_dir, 'config', 'params.yaml')

    default_model = os.path.expanduser(
        '~/.ros/uav_depth_track_nn/model/model.pth')

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
        DeclareLaunchArgument(
            'radius', default_value='3.0', description='给定圆轨迹半径（米）'),
        DeclareLaunchArgument(
            'height', default_value='2.5', description='巡航高度（米）'),
    ]

    airsim_source = Node(
        package='uav_depth_track_nn',
        executable='airsim_source',
        name='airsim_source',
        namespace='uav',
        output='screen',
        parameters=[params_file, {
            'airsim_ip': ParameterValue(LaunchConfiguration('airsim_ip'), value_type=str),
            'airsim_port': ParameterValue(LaunchConfiguration('airsim_port'), value_type=int),
            'vehicle_name': ParameterValue(LaunchConfiguration('vehicle_name'), value_type=str),
        }],
        remappings=UAV_TOPIC_REMAPS + UAV_SERVICE_REMAPS,
    )

    tracker = Node(
        package='uav_depth_track_nn',
        executable='main',
        name='uav_depth_track',
        output='screen',
        parameters=[params_file, {
            'model_path': ParameterValue(LaunchConfiguration('model_path'), value_type=str),
            'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=float),
        }],
    )

    return LaunchDescription(arguments + [airsim_source, tracker])
