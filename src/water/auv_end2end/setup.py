from setuptools import setup
from glob import glob
import os

package_name = 'auv_end2end'

setup(
    name=package_name,
    version='0.1.0',
    maintainer='liahn',
    maintainer_email='xxxuuaa@users.noreply.github.com',
    license='MIT',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'auv_obstacle_avoid = auv_end2end.auv_obstacle_avoid:main',
            'auv_rect_cruise = auv_end2end.auv_rect_cruise:main',
            'trajectory_recorder = auv_end2end.trajectory_recorder:main',
        ],
    },
)
