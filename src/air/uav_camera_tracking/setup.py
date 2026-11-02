import os
from glob import glob

from setuptools import setup

package_name = 'uav_camera_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 预训练权重：节点用 ament share 目录定位，install 后无需改路径
        ('share/' + package_name + '/models', glob('models/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6lebron23',
    maintainer_email='3248146937@qq.com',
    description='无人机机载相机视觉目标跟踪（感知网络 + 控制网络 + 端到端对比）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = uav_camera_tracking.main:main',
            'sim = uav_camera_tracking.sim_node:main',
            'detector = uav_camera_tracking.detector_node:main',
            'controller = uav_camera_tracking.controller_node:main',
        ],
    },
)
