#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置（colcon build 用）."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_hover_wind_limit'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'README.md', 'main.py',
                                               'requirements.txt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xy18574557330',
    maintainer_email='3909057723@qq.com',
    description='悬停抗风能力核算：倾角、推重比余量与蒲福风级（ROS 2 Humble）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wind_limit = uav_hover_wind_limit.cli:main',
            'wind_limit_node = uav_hover_wind_limit.ros_node:main',
        ],
    },
)
