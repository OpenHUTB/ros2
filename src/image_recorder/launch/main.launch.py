from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    output_dir = LaunchConfiguration('output_dir')
    image_format = LaunchConfiguration('image_format')
    show_window = LaunchConfiguration('show_window')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/carla/ego_vehicle/camera/rgb/front/image',
            description='要订阅的图像话题'),
        DeclareLaunchArgument(
            'output_dir', default_value='.',
            description='图片保存目录（不存在时自动创建）'),
        DeclareLaunchArgument(
            'image_format', default_value='jpg',
            description='保存格式：jpg / jpeg / png'),
        DeclareLaunchArgument(
            'show_window', default_value='true',
            description='是否弹窗显示；无图形界面时设为 false，逐帧保存'),
        Node(
            package='image_recorder',
            executable='main',
            name='image_recorder_node',
            output='screen',
            parameters=[{
                'image_topic': ParameterValue(image_topic, value_type=str),
                'output_dir': ParameterValue(output_dir, value_type=str),
                'image_format': ParameterValue(image_format, value_type=str),
                'show_window': ParameterValue(show_window, value_type=bool),
            }]
        )
    ])
