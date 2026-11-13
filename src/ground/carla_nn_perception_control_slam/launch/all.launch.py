from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('carla_nn_perception_control_slam')
    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('carla_port', default_value='2000'),
        *[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', f'task{i}.launch.py')),
                launch_arguments={'carla_host': LaunchConfiguration('carla_host'),
                                  'carla_port': LaunchConfiguration('carla_port')}.items(),
            )
            for i in (1, 2, 3, 4)
        ],
    ])
