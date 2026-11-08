from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_link_aware_degradation'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='zqhuang76',
    maintainer_email='zqhuang76@users.noreply.github.com',
    description='Link-quality-aware degradation and failsafe return for UAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = uav_link_aware_degradation.cli:main',
            'link_emulator = uav_link_aware_degradation.ros.link_emulator_node:main',
            'supervisor = uav_link_aware_degradation.ros.supervisor_node:main',
        ],
    },
)
