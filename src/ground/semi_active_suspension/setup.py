from glob import glob

from setuptools import setup

package_name = 'semi_active_suspension'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='李英杰',
    maintainer_email='327110089+lzxbaba@users.noreply.github.com',
    description='半主动悬架车身姿态控制仿真',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semi_active_suspension = semi_active_suspension.main:main',
            'semi_active_suspension_node = semi_active_suspension.ros_node:main',
        ],
    },
)
