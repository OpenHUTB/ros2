import os
from glob import glob

from setuptools import setup

package_name = 'carla_benchmark_suite'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himyinjun1234',
    maintainer_email='1636550364@qq.com',
    description='CARLA 0.9.16 作业综合整合与性能评价 ROS 2 功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'benchmark_node = carla_benchmark_suite.benchmark_node:main',
        ],
    },
)
