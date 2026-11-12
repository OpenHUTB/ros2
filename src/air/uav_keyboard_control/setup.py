from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_keyboard_control'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uav',
    maintainer_email='uav@example.com',
    description='Terminal keyboard -> body-frame /uav/cmd_vel (ROS2 Humble)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard = uav_keyboard_control.keyboard_node:main',
        ],
    },
)
