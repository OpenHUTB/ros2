#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置（colcon build 用）."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'diff_drive_powertrain_sizing'

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
    description='差速底盘驱动系统选型核算：滚动/坡度/加速/风阻阻力，折算轮边转矩、轮速与功率，对比直接驱动与减速箱方案',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_powertrain_sizing = diff_drive_powertrain_sizing.cli:main',
        ],
    },
)
