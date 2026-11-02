from glob import glob

from setuptools import setup

package_name = 'parking_path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='李英杰',
    maintainer_email='2535030016@stu.hutb.edu.cn',
    description='垂直车位泊车路径规划（Hybrid A*）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'parking_path_planner = parking_path_planner.ros_node:main',
        ],
    },
)
