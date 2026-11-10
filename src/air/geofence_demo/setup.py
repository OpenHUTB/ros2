import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'geofence_demo'

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
    maintainer='Toeting',
    maintainer_email='328915386@qq.com',
    description='基于 ROS 2 的禁飞区动态检测与安全预警模块',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'geofence_node = geofence_demo.geofence_node:main',
        ],
    },
)
