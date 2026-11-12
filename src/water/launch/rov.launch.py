import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_water = get_package_share_directory('water')
    world_file = os.path.join(pkg_water, 'worlds', 'underwater.world')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='是否启动 Gazebo 图形界面')
    declare_image_topic = DeclareLaunchArgument(
        'image_topic', default_value='/sonar/camera/image_raw',
        description='与 underwater.world 中相机实际发布的话题保持一致')

    # gazebo_ros 自带的 gazebo.launch.py，通过 world 参数加载水下场景
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration('gui'),
        }.items()
    )

    infer_node = Node(
        package='water',
        executable='sonar_infer_node',
        name='sonar_infer_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
        }],
    )

    return LaunchDescription([
        declare_gui,
        declare_image_topic,
        gazebo,
        infer_node,
    ])
