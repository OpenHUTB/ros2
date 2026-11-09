from setuptools import setup
import os
from glob import glob

package_name = 'carla_parking_lot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lyzzzz0921',
    maintainer_email='259402172@qq.com',
    description='CARLA Town04 parking lot scene generation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = carla_parking_lot.parking_lot_node:main',
        ],
    },
)
