from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_e2e_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Enemy52',
    maintainer_email='enemy52@example.com',
    description='End-to-end visual navigation with CNN+PPO on ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deploy = uav_e2e_rl.deploy_node:main',
            'train = uav_e2e_rl.train_env:main',
        ],
    },
)
