import os
from glob import glob

from setuptools import setup

package_name = 'uav_depth_track_nn'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dniuna',
    maintainer_email='dniuna@example.com',
    description='AirSim 无人机深度相机感知 + 给定轨迹跟踪（ROS 2 Humble）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = uav_depth_track_nn.main:main',
            'collect = uav_depth_track_nn.collect:main',
            'train = uav_depth_track_nn.train:main',
            'airsim_source = uav_depth_track_nn.airsim_source:main',
        ],
    },
)
