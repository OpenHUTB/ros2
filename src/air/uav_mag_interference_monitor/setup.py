from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_mag_interference_monitor'

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
    description='UAV magnetometer interference check and heading consistency gate',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mag_interference_node = '
            'uav_mag_interference_monitor.mag_interference_node:main',
        ],
    },
)
