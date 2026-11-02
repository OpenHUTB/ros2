from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_push_recovery'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # MuJoCo 模型文件
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='L2008-by',
    maintainer_email='L2008-by@users.noreply.github.com',
    description='人形机器人平衡与外力抗扰恢复 ROS2 功能包（ZMP/捕获点 + 踝/髋策略）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'balance_sim_node = humanoid_push_recovery.mujoco_sim_node:main',
            'push_teleop_node = humanoid_push_recovery.push_teleop_node:main',
        ],
    },
)
