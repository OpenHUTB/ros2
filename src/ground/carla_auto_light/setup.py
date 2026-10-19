from glob import glob
import os

from setuptools import setup

package_name = 'carla_auto_light'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'main.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='openhutb@example.com',
    description='CARLA 地面载具灯光自动控制：自动大灯、雾灯、转向灯（回正自熄灭）、刹车灯',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_light_node = carla_auto_light.light_node:main',
        ],
    },
)
