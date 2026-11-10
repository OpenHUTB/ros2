from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_temporal_e2e'

setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
        (os.path.join('share', package_name, 'models_trained'), glob('models_trained/*.npz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='WZJ6666wudu', maintainer_email='WZJ6666wudu@users.noreply.github.com',
    description='人形机器人多帧时序端到端视觉平衡控制',
    license='MIT',
    entry_points={
        'console_scripts': [
            'temporal_sim_node = humanoid_temporal_e2e.sim_node:main',
            'temporal_predictor_node = humanoid_temporal_e2e.predictor_node:main',
        ],
    },
)
