from setuptools import setup
from glob import glob
import os

package_name = 'auv_end2end'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
        data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 自动安装launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 自动安装world场景文件
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # 自动把节点安装到 lib 标准路径
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'auv_obstacle_avoid = auv_end2end.auv_obstacle_avoid:main',
        ],
    },
)

