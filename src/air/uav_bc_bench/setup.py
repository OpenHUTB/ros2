import os
from glob import glob

from setuptools import setup

package_name = 'uav_bc_bench'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dniuna',
    maintainer_email='2042189022@qq.com',
    description='行为克隆模型对比基准',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = uav_bc_bench.main:main',
            'bench = uav_bc_bench.bench:main',
        ],
    },
)
