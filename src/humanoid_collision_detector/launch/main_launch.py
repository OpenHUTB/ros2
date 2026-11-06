"""
collision_detection.launch.py
起动 humanoid_collision_detector + obstacle_demo_node
(当前模块只支持静态简化碰撞体，不依赖 robot_state_publisher / TF)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_safety_boundary = DeclareLaunchArgument(
        'safety_boundary', default_value='0.15')
    declare_obstacle_radius = DeclareLaunchArgument(
        'obstacle_radius', default_value='0.2')
    declare_check_freq = DeclareLaunchArgument(
        'check_frequency', default_value='100.0')

    # 碰撞检测节点（核心）
    collision_detector = Node(
        package='humanoid_collision_detector',
        executable='collision_detector_node',
        name='humanoid_collision_detector',
        parameters=[{
            'safety_boundary': LaunchConfiguration('safety_boundary'),
            'obstacle_radius': LaunchConfiguration('obstacle_radius'),
            'check_frequency': LaunchConfiguration('check_frequency'),
        }],
        output='screen',
    )

    # 障碍物演示节点（正弦轨迹）
    obstacle_demo = Node(
        package='humanoid_collision_detector',
        executable='obstacle_demo_node',
        name='obstacle_demo_node',
        parameters=[{
            'origin_x': 0.85,
            'origin_y': 0.0,
            'origin_z': 1.15,
            'amplitude': 0.55,
            'frequency': 1.0,
            'publish_hz': 20.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_safety_boundary,
        declare_obstacle_radius,
        declare_check_freq,
        collision_detector,
        obstacle_demo,
    ])
