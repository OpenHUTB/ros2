import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('auv_end2end'),
        'worlds',
        'underwater.world'
    )

    return LaunchDescription([
        # 启动Gazebo仿真
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen',
            name='gazebo_sim'
        ),
        # 启动视觉避障节点
        Node(
            package='auv_end2end',
            executable='auv_obstacle_avoid',
            name='auv_obstacle_avoid',
            output='screen'
        )
    ])

