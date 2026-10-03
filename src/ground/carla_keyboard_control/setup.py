import os
from glob import glob

from setuptools import setup

package_name = 'carla_keyboard_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件（ROS 2 的 *.launch.py 与 ROS 1 的 *.launch）
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.launch')),
        # 配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himyinjun1234',
    maintainer_email='1636550364@qq.com',
    description='CARLA 0.9.16 地面载具物理仿真与键盘运动控制 ROS 2 功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_control_node = carla_keyboard_control.carla_control_node:main',
            'keyboard_teleop_node = carla_keyboard_control.keyboard_teleop_node:main',
        ],
    },
)
