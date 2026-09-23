from setuptools import setup
import os
from glob import glob

package_name = 'rov_mujoco'

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
        # MuJoCo 模型文件与配置
        (os.path.join('share', package_name, 'models'), glob('models/*.xml') + glob('models/*.json')),
        (os.path.join('share', package_name, 'models', 'meshes', 'ur5e'), glob('models/meshes/ur5e/*')),
        (os.path.join('share', package_name, 'models', 'meshes', 'robotiq'), glob('models/meshes/robotiq/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shark',
    maintainer_email='syysean@github.com',
    description='水下机器人 MuJoCo 仿真 ROS2 功能包',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = rov_mujoco.mujoco_sim_node:main',
            'keyboard_teleop_node = rov_mujoco.keyboard_teleop_node:main',
        ],
    },
)
