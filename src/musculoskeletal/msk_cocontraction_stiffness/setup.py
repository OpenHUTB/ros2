#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置（colcon build 用）."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'msk_cocontraction_stiffness'

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
    description='拮抗肌共收缩与关节刚度核算：两侧肌力反解、关节刚度、扰动偏转与共收缩上限',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'msk_cocontraction_stiffness = msk_cocontraction_stiffness.cli:main',
        ],
    },
)
