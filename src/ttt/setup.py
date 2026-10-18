from setuptools import setup
import os
from glob import glob

package_name = 'geofence_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 这一行非常重要，它让系统能找到 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='基于 ROS 2 的禁飞区动态检测与安全预警系统',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式：'你想用的命令名 = 包名.文件名:main函数'
            'geofence_node = geofence_detector.geofence_node:main',
        ],
    },
)
