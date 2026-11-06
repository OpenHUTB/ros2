from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_motor_thrust_imbalance_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB contributors',
    maintainer_email='contributors@example.com',
    description='UAV motor thrust imbalance detection and emergency landing trigger',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_imbalance_node = '
            'uav_motor_thrust_imbalance_monitor.motor_imbalance_node:main',
        ],
    },
)
