import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('water_vision_enhancement')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    turbidity_arg = DeclareLaunchArgument(
        'turbidity', default_value='0.5',
        description='模拟水体浑浊度 0~1')

    tracker_arg = DeclareLaunchArgument(
        'tracker_type', default_value='KCF',
        description='追踪器类型 KCF / CSRT / MOSSE')

    # 演示节点：模拟浑浊水体视频
    video_demo_node = Node(
        package='water_vision_enhancement',
        executable='video_demo_node',
        name='water_video_demo_node',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 20,
            'turbidity': LaunchConfiguration('turbidity'),
            'target_speed': 80.0,
            'output_topic': '/camera/image_raw',
        }],
    )

    # 主节点：图像增强 + 目标追踪
    vision_node = Node(
        package='water_vision_enhancement',
        executable='vision_node',
        name='water_vision_node',
        output='screen',
        parameters=[
            params_file,
            {'tracker_type': LaunchConfiguration('tracker_type')},
        ],
    )

    return LaunchDescription([
        turbidity_arg,
        tracker_arg,
        video_demo_node,
        vision_node,
    ])
