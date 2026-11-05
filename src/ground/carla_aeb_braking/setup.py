import os
from glob import glob

from setuptools import setup

package_name = 'carla_aeb_braking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.launch')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='825404295',
    maintainer_email='825404295@qq.com',
    description='CARLA 0.9.16 自动紧急制动（AEB）ROS 2 功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aeb_bridge_node = carla_aeb_braking.bridge_node:main',
            'aeb_controller_node = carla_aeb_braking.aeb_node:main',
        ],
    },
)
