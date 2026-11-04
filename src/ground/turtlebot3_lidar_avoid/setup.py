import os
from glob import glob

from setuptools import setup

package_name = 'turtlebot3_lidar_avoid'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件（ROS 2 使用 *.launch.py）
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 参数配置
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xjm666god',
    maintainer_email='xjm666god@users.noreply.github.com',
    description='TurtleBot3 激光雷达自主避障 ROS 2 功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_node = turtlebot3_lidar_avoid.avoid:main',
        ],
    },
)
