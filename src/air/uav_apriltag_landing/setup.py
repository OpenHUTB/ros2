from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_apriltag_landing'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xu-da111',
    maintainer_email='xu-da111@example.com',
    description='UAV AprilTag visual landing module',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_landing_node = uav_apriltag_landing.apriltag_landing_node:main',
        ],
    },
)
