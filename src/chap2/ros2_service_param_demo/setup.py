from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ros2_service_param_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gao2006-12',
    maintainer_email='gao2006-12@users.noreply.github.com',
    description='ROS2 服务与参数基础实验（无人机模式切换与在线调参）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_mode_node = ros2_service_param_demo.uav_mode_node:main',
        ],
    },
)
