from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carla_rl_driving'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 与 config 随包安装，支持 ros2 launch 直接拉起
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='openhutb@example.com',
    description='CARLA end-to-end reinforcement-learning driving (image -> policy -> ackermann)',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 部署推理节点：订阅相机图像 -> 加载训练好的策略 -> 发布阿克曼控制
            'infer_node = carla_rl_driving.infer_node:main',
            # 训练入口（可在终端直接 python -m 或 ros2 run 调用）
            'train = carla_rl_driving.train:main',
        ],
    },
)
