"""安装无人机定高日志误差与恢复时间评估."""
from glob import glob
from setuptools import find_packages, setup

setup(
    name='uav_hover_log_eval', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/uav_hover_log_eval']),
        ('share/uav_hover_log_eval', ['package.xml']),
        ('share/uav_hover_log_eval/launch', glob('launch/*.launch.py')),
        ('share/uav_hover_log_eval/models', glob('models/*.xml')),
        ('share/uav_hover_log_eval/data', glob('data/*')),
    ],
    install_requires=['setuptools'], zip_safe=False,
    description='无人机定高日志误差与恢复时间评估', license='MIT',
    entry_points={'console_scripts': ['main = uav_hover_log_eval.cli:main', 'node = uav_hover_log_eval.ros_node:main']},
)
