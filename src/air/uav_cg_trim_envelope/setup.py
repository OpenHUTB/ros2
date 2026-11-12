#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置（colcon build 用）."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_cg_trim_envelope'

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
    description='多旋翼重心偏移配平与饱和包线核算：四电机悬停拉力分配、配平倾角、饱和包线与裕度',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_cg_trim_envelope = uav_cg_trim_envelope.cli:main',
        ],
    },
)
