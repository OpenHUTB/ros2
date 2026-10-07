from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', 'main.py'],
            cwd='/home/dniuna/ros2/src/uav_mujoco_keyboard',
            output='screen'
        )
    ])
