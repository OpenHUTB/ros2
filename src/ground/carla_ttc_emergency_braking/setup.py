import os
from glob import glob

from setuptools import setup

package_name = 'carla_ttc_emergency_braking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py') + glob('launch/*.launch')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenxi',
    maintainer_email='cx3238842481@qq.com',
    description='OpenHUTB simulation experiment: ' + package_name,
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_ttc_emergency_braking = carla_ttc_emergency_braking.app:main',
        ],
    },
)