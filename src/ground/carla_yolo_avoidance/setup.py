"""ament_python 构建脚本。

ROS 2 Humble 使用 Python 3.10，`colcon build` 时会调用本文件安装包体、
launch 文件、RViz 配置以及 console_scripts 入口。
"""

import os
from glob import glob

from setuptools import setup

package_name = 'carla_yolo_avoidance'

setup(
    name=package_name,
    version='1.0.0',
    # 包体目录：carla_yolo_avoidance/ 下的所有 Python 模块
    packages=[package_name],
    data_files=[
        # ament 索引需要的资源标记文件
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 必须安装 package.xml，否则 ros2 run / ros2 launch 找不到包
        (os.path.join('share', package_name), ['package.xml']),
        # launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # RViz 可视化配置
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # ROS 参数文件（CARLA 的 IP / 端口等在这里配置）
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB Student',
    maintainer_email='openhutb_course@example.com',
    description='CARLA + YOLOv8 + ROS 2 Humble 目标检测与减速避障示例',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 统一入口（推荐）：ros2 run carla_yolo_avoidance main --node <camera|yolo|decision>
            'main = carla_yolo_avoidance.main:main',
            # 三个独立节点的独立入口，launch 文件使用这三个
            'carla_camera_node = carla_yolo_avoidance.main:main_camera',
            'yolo_detect_node = carla_yolo_avoidance.main:main_yolo',
            'decision_control_node = carla_yolo_avoidance.main:main_decision',
        ],
    },
)
