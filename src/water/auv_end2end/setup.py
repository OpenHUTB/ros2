from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_end2end'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        # ament 资源索引文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml 元数据
        ('share/' + package_name, ['package.xml']),
        # 批量安装所有 launch 启动文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 批量安装所有 worlds 场景文件
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # 安装节点可执行脚本到 libexec 目录
        ('lib/' + package_name, ['auv_end2end/auv_rect_cruise.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'auv_obstacle_avoid = auv_end2end.auv_obstacle_avoid:main',
            'auv_rect_cruise = auv_end2end.auv_rect_cruise:main',
        ],
    },
)

