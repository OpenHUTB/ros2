from glob import glob
import os

from setuptools import setup

package_name = 'carla_lidar_pointcloud_segmentation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'main.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='openhutb@example.com',
    description='CARLA 激光雷达点云语义分割：语义激光雷达自监督 + 逐点分类网络',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_segmentation_node = carla_lidar_pointcloud_segmentation.node:main',
        ],
    },
)