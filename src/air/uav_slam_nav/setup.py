# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_slam_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dniuna',
    maintainer_email='dniuna@example.com',
    description='AirSim 无人机建图与自主导航（ROS2 Humble）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 唯一入口：用 --role 选择节点组合，默认 all 启动全套
            'main = uav_slam_nav.main:main',
        ],
    },
)
