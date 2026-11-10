from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_local_planner_bench'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='L2008-by',
    maintainer_email='L2008-by@users.noreply.github.com',
    description='人形足迹下的局部避障算法对比（DWA / APF）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_bench_node = humanoid_local_planner_bench.ros_node:main',
        ],
    },
)
