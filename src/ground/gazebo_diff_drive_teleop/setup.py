import os
from glob import glob
from setuptools import setup

package_name = 'gazebo_diff_drive_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='夏睿泽',
    maintainer_email='q5418374071104@outlook.com',
    description='Differential-drive robot keyboard teleop in Gazebo Harmonic (ROS2 Jazzy)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = gazebo_diff_drive_teleop.teleop_keyboard:main',
            'auto_drive = gazebo_diff_drive_teleop.auto_drive:main',
        ],
    },
)
