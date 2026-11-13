import os
from glob import glob

from setuptools import setup

package_name = 'vehicle_end_to_end_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.launch')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wqs159',
    maintainer_email='wqs159@example.com',
    description='规则式 AEB 自动紧急制动演示系统（Pygame，射线测距 + 三级规则状态机，非图像端到端学习模型）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_sim = vehicle_end_to_end_sim.main:main',
        ],
    },
)
