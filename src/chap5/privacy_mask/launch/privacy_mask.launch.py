import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory("privacy_mask")

    node = Node(
        package="privacy_mask",
        # 关键修复：必须与 setup.py 中 console_scripts 的可执行名完全一致
        executable="privacy_mask_main",
        parameters=[{
            # 与 resource/ 中实际文件名保持一致（原文件写成了 test.jpg）
            "image_path": os.path.join(pkg_dir, "resource", "test.png"),
            "model_path": os.path.join(pkg_dir, "resource", "yolov8n.pt"),
        }],
        output="screen",
    )
    return LaunchDescription([node])
