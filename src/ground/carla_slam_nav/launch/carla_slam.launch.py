"""SLAM 启动：ros2 launch carla_slam_nav carla_slam.launch.py

用 RTAB-Map 对 CARLA 激光雷达做 6 位姿 SLAM 建图，输出 /map 与 odom->base 融合 TF。
前置：CARLA + ros-bridge 已生成带激光雷达的 ego 车辆。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    slam_dir = os.path.join(get_package_share_directory('rtabmap_ros'), 'launch')
    config = os.path.join(
        get_package_share_directory('carla_slam_nav'), 'config', 'rtabmap_lidar.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic',
                              default_value='/carla/ego_vehicle/lidar'),
        DeclareLaunchArgument('odom_topic',
                              default_value='/carla/ego_vehicle/odometry'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_dir, 'rtabmap.launch.py')),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'database_path': '~/.ros/rtabmap_carla.db',
                'queue_size': '10',
                'frame_id': 'base_footprint',
                'map_frame': 'map',
                'odom_topic': LaunchConfiguration('odom_topic'),
                'scan_topic': '',                 # 无 2D scan，用 3D 点云
                'subscription_topic': LaunchConfiguration('lidar_topic'),
                'visual_odometry': 'false',
                'sync_queue_size': '20',
            }.items(),
        ),
    ])
