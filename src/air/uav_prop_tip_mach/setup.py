#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置（colcon build 用）."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_prop_tip_mach'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'README.md', 'main.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xy18574557330',
    maintainer_email='3909057723@qq.com',
    description='旋翼桨尖马赫数与桨盘载荷核算（纯标准库，ROS 2 launch 包装）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prop_tip_mach = uav_prop_tip_mach.cli:main',
        ],
    },
)
