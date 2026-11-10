from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_fall_protection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='L2008-by',
    maintainer_email='L2008-by@users.noreply.github.com',
    description='人形机器人跌倒预判与保护性倒地',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fall_protection_node = humanoid_fall_protection.ros_node:main',
        ],
    },
)
