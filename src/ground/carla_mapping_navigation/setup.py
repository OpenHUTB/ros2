import os
from glob import glob

from setuptools import setup

package_name = 'carla_mapping_navigation'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himyinjun1234',
    maintainer_email='1636550364@qq.com',
    description='CARLA 0.9.16 占用栅格建图与神经网络规划导航 ROS 2 功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_navigation_node = carla_mapping_navigation.mapping_navigation_node:main',
        ],
    },
)
