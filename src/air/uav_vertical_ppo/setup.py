"""安装四旋翼垂直通道 PPO 定高控制."""
from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

setup(
    name='uav_vertical_ppo', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/uav_vertical_ppo']),
        ('share/uav_vertical_ppo', ['package.xml']),
        ('share/uav_vertical_ppo/launch', glob('launch/*.launch.py')),
        ('share/uav_vertical_ppo/models', glob('models/*.xml')),
        ('share/uav_vertical_ppo/data/evaluation', glob('data/evaluation/*')),
        ('share/uav_vertical_ppo/data', [p for p in glob('data/*') if Path(p).is_file()]),
    ],
    install_requires=['setuptools'], zip_safe=False,
    description='四旋翼垂直通道 PPO 定高控制', license='MIT',
    entry_points={'console_scripts': ['main = uav_vertical_ppo.cli:main', 'node = uav_vertical_ppo.ros_node:main']},
)
