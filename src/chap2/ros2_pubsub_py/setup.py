from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ros2_pubsub_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gao2006-12',
    maintainer_email='gao2006-12@users.noreply.github.com',
    description='ROS2 话题发布订阅（talker/listener）基础实验，用于无人机项目通信前置验证',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros2_pubsub_py.talker:main',
            'listener = ros2_pubsub_py.listener:main',
        ],
    },
)
