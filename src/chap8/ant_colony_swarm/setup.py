import os
from setuptools import find_packages, setup

package_name = 'ant_colony_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            [os.path.join('launch', 'swarm_bringup.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xavier-101666',
    maintainer_email='2754586324@qq.com',
    description='基于机器视觉的多机器人蚁群编队仿真',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_detector = ant_colony_swarm.vision_detect_node:main',
            'swarm_control = ant_colony_swarm.swarm_control_node:main',
        ],
    },
)
