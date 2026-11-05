import os
from glob import glob

from setuptools import setup

package_name = 'uav_vision_nn'

setup(
    name=package_name,
    version='0.2.0',
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
    description='AirSim 无人机视觉端到端控制（ROS2 Humble）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = uav_vision_nn.main:main',
            'collect = uav_vision_nn.collect:main',
            'train = uav_vision_nn.train:main',
            'airsim_source = uav_vision_nn.airsim_source:main',
        ],
    },
)
