import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('privacy_mask')

    # 只把用户在命令行显式传入（非空）的参数交给节点，其余走节点自带默认值
    parameters = {}
    image_path = LaunchConfiguration('image_path').perform(context)
    if image_path:
        parameters['image_path'] = image_path
    else:
        parameters['image_path'] = os.path.join(
            pkg_dir, 'resource', 'test.png')

    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path:
        parameters['model_path'] = model_path
    else:
        parameters['model_path'] = os.path.join(
            pkg_dir, 'resource', 'yolov8n.pt')

    blur_kernel = LaunchConfiguration('blur_kernel').perform(context)
    if blur_kernel:
        parameters['blur_kernel'] = int(blur_kernel)

    publish_period = LaunchConfiguration('publish_period').perform(context)
    if publish_period:
        parameters['publish_period'] = float(publish_period)

    node = Node(
        package='privacy_mask',
        executable='privacy_mask_main',
        parameters=[parameters],
        output='screen',
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_path', default_value='',
            description='测试图片路径，留空用包内 resource/test.png'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='权重路径，留空用包内 resource/yolov8n.pt'),
        DeclareLaunchArgument(
            'blur_kernel', default_value='',
            description='高斯核大小，留空或 0 表示自适应'),
        DeclareLaunchArgument(
            'publish_period', default_value='',
            description='发布周期（秒），留空用 0.1'),
        OpaqueFunction(function=launch_setup),
    ])
