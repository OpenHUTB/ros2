#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置（colcon build 用）."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'usv_froude_scaling'

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
    description='USV 弗劳德缩比换算：船模-实船速度、阻力与功率换算，ITTC 1957 分解法估算实船总阻力与拖曳功率',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_froude_scaling = usv_froude_scaling.cli:main',
        ],
    },
)
