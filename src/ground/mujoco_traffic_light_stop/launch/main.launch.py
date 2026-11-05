import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_traffic_light_stop')
    main_script = os.path.join(pkg_share, 'main.py')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', main_script],
            output='screen'
        )
    ])
