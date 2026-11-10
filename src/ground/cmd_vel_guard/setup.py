from glob import glob
import os

from setuptools import setup

package_name = 'cmd_vel_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='234208483+gfdklzr@users.noreply.github.com',
    description='速度指令安全监督节点：限幅、急停、超时与非法值拦截',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_guard = cmd_vel_guard.cmd_vel_guard_node:main',
        ],
    },
)
