from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_human_following'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='L2008-by',
    maintainer_email='L2008-by@users.noreply.github.com',
    description='人形机器人 MuJoCo 人体目标跟随仿真 ROS2 功能包',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = humanoid_human_following.mujoco_sim_node:main',
        ],
    },
)
