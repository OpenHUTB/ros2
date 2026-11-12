from glob import glob
import os

from setuptools import setup

package_name = 'carla_visual_lane_departure_warning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'main.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='openhutb@example.com',
    description='相机版车道偏离预警：视觉车道线检测 + 地图真值对照',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_lane_departure_node = carla_visual_lane_departure_warning.node:main',
        ],
    },
)
