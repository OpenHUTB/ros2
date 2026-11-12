"""采集 launch：AirSim 桥 + 专家示教采集节点。

用法：
    ros2 launch uav_depth_track_nn collect.launch.py \\
        airsim_port:=8080 vehicle_name:=drone1 laps:=2.0

采集结束后采集节点会自动请求降落并退出，AirSim 桥保持运行，Ctrl-C 结束。
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

    default_data_dir = os.path.expanduser('~/.ros/uav_depth_track_nn/data')

    arguments = [
        DeclareLaunchArgument(
            'airsim_ip', default_value='10.0.2.2',
            description='AirSim 所在主机地址'),
        DeclareLaunchArgument(
            'airsim_port', default_value='41451',
            description='AirSim RPC 端口（settings.json 的 ApiServerPort）'),
        DeclareLaunchArgument(
            'vehicle_name', default_value='',
            description='AirSim 载具名，默认载具留空'),
        DeclareLaunchArgument(
            'data_dir', default_value=default_data_dir,
            description='样本输出目录'),
        DeclareLaunchArgument(
            'radius', default_value='3.0', description='圆轨迹半径（米）'),
        DeclareLaunchArgument(
            'height', default_value='2.5', description='巡航高度（米）'),
        DeclareLaunchArgument(
            'laps', default_value='2.0', description='采集圈数'),
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

    collector = Node(
        package='uav_depth_track_nn',
        executable='collect',
        name='uav_depth_collect',
        output='screen',
        parameters=[params_file, {
            'data_dir': ParameterValue(LaunchConfiguration('data_dir'), value_type=str),
            'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=float),
            'laps': ParameterValue(LaunchConfiguration('laps'), value_type=float),
        }],
    )

    return LaunchDescription(arguments + [airsim_source, collector])
