from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("privacy_mask")
    img_path = os.path.join(pkg_dir, "resource", "test.jpg")

    node = Node(
        package="privacy_mask",
        executable="privacy_mask_main",
        parameters=[
            {"image_path": img_path}
        ],
        output="screen"
    )
    return LaunchDescription([node])
