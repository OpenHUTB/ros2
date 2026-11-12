#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置（colcon build 用）."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'humanoid_support_margin'

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
    maintainer='Hilin-67',
    maintainer_email='328919000+Hilin-67@users.noreply.github.com',
    description='人形机器人支撑多边形与倾倒裕度核算：双足凸包、稳定裕度、临界推力/冲量与足底载荷分配',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_support_margin = humanoid_support_margin.cli:main',
        ],
    },
)
