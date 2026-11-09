from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_slam_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Enemy52',
    maintainer_email='enemy52@example.com',
    description='UAV SLAM mapping and navigation on ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'points_to_scan = uav_slam_nav.points_to_scan_node:main',
            'odom_tf = uav_slam_nav.odom_tf_node:main',
            'simple_nav = uav_slam_nav.simple_nav_node:main',
            'nav_relay = uav_slam_nav.nav_relay_node:main',
            'main = uav_slam_nav.cli:main',
        ],
    },
)
