from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_camera_e2e_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # MuJoCo 模型文件
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
        # 训练权重（随包提交，便于直接复现推理演示）
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lb-lt',
    maintainer_email='lb-lt@users.noreply.github.com',
    description='人形机器人单目相机图像端到端神经网络控制 ROS2 功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = humanoid_camera_e2e_control.mujoco_sim_node:main',
            'e2e_policy_node = humanoid_camera_e2e_control.e2e_policy_node:main',
            'keyboard_teleop_node = humanoid_camera_e2e_control.keyboard_teleop_node:main',
        ],
    },
)
