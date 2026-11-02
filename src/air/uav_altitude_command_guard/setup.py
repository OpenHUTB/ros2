"""安装无人机定高指令限幅与超时保持."""
from glob import glob
from setuptools import find_packages, setup

setup(
    name='uav_altitude_command_guard', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/uav_altitude_command_guard']),
        ('share/uav_altitude_command_guard', ['package.xml']),
        ('share/uav_altitude_command_guard/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=False,
    tests_require=['pytest'],
    description='无人机定高指令限幅与超时保持', license='MIT',
    entry_points={'console_scripts': ['main = uav_altitude_command_guard.cli:main', 'node = uav_altitude_command_guard.ros_node:main']},
)
