from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_water = get_package_share_directory('water')
    world_file = os.path.join(pkg_water, 'worlds', 'underwater.world')

    # 启动Gazebo，加载水下场景
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # 启动CNN声呐识别节点
    infer_node = Node(
        package='water',
        executable='sonar_infer_node',
        name='sonar_infer_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        infer_node
    ])
