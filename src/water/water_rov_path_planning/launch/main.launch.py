import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('water_rov_path_planning')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    current_pattern_arg = DeclareLaunchArgument(
        'current_pattern', default_value='vortex',
        description='洋流模式：uniform / vortex / tidal / shear')

    current_magnitude_arg = DeclareLaunchArgument(
        'current_magnitude', default_value='0.4',
        description='基础洋流流速（m/s）')

    # 规划节点
    planning_node = Node(
        package='water_rov_path_planning',
        executable='planning_node',
        name='rov_planning_node',
        output='screen',
        parameters=[
            params_file,
            {
                'current_pattern': LaunchConfiguration('current_pattern'),
                'current_magnitude': LaunchConfiguration('current_magnitude'),
            },
        ],
    )

    # 演示节点
    demo_node = Node(
        package='water_rov_path_planning',
        executable='demo_node',
        name='rov_demo_node',
        output='screen',
        parameters=[{
            'world_size': 50.0,
            'num_obstacles': 12,
            'start_x': 5.0,
            'start_y': 25.0,
            'goal_x': 45.0,
            'goal_y': 25.0,
            'obstacle_seed': 42,
        }],
    )

    return LaunchDescription([
        current_pattern_arg,
        current_magnitude_arg,
        planning_node,
        demo_node,
    ])
