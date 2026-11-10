from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mean_filter_pkg'

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
    maintainer_email='toeting@users.noreply.github.com',
    description='ROS 2 滑动窗口均值滤波节点，用于传感器数据平滑',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mean_filter = mean_filter_pkg.mean_filter_node:main',
        ],
    },
)
