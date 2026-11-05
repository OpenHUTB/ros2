# -*- coding: utf-8 -*-
"""ament_python 安装脚本：console_scripts 是 ROS2 的入口约定。"""

from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_airsim_keyboard'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dniuna',
    maintainer_email='dniuna@example.com',
    description='AirSim 四旋翼无人机键盘遥控与 ROS2 桥接',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 模块总入口：一条命令跑起整个模块（桥接 + 键盘）
            'main = uav_airsim_keyboard.main:main',
            # 只跑键盘发布器：ros2 launch 不转发 stdin，需要另开终端时用
            'keyboard = uav_airsim_keyboard.keyboard:main',
            'bridge = uav_airsim_keyboard.bridge:main',
        ],
    },
)
