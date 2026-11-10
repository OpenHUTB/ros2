from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_vision_fall_prediction'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
        # 训练好的权重必须随包安装，否则节点启动即 FileNotFoundError
        (os.path.join('share', package_name, 'models_trained'),
         glob('models_trained/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WZJ6666wudu',
    maintainer_email='WZJ6666wudu@users.noreply.github.com',
    description='人形机器人视觉跌倒预测与分级平衡控制',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_sim_node = humanoid_vision_fall_prediction.vision_sim_node:main',
            'fall_predictor_node = humanoid_vision_fall_prediction.fall_predictor_node:main',
        ],
    },
)
