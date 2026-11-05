from glob import glob
import os

from setuptools import setup

package_name = 'carla_nn_car_following'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='openhutb@example.com',
    description='基于神经网络的自适应跟车（ACC）仿真模块',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'acc_scene_node = carla_nn_car_following.acc_scene_node:main',
        ],
    },
)
