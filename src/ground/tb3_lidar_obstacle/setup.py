import os
from glob import glob

from setuptools import setup

package_name = 'tb3_lidar_obstacle'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mjm-peter',
    maintainer_email='Mjm-peter@users.noreply.github.com',
    description='TurtleBot3 激光雷达避障 ROS 2 功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoid = tb3_lidar_obstacle.obstacle_avoid:main',
        ],
    },
)
