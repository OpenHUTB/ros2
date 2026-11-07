from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holoocean_depth_avoid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ri-kin08',
    maintainer_email='CHANGE_ME@example.com',
    description='基于深度图的水下机器人前视避障与海底定高巡航',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv_depth_avoid = holoocean_depth_avoid.main:main',
        ],
    },
)
