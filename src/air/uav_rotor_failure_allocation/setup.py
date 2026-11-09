"""安装四旋翼单桨失效推力分配与评估."""
from glob import glob
from setuptools import find_packages, setup

setup(
    name='uav_rotor_failure_allocation', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/uav_rotor_failure_allocation']),
        ('share/uav_rotor_failure_allocation', ['package.xml']),
        ('share/uav_rotor_failure_allocation/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=False,
    tests_require=['pytest'],
    description='四旋翼单桨失效后的推力分配可行域与姿态权限评估', license='MIT',
    entry_points={'console_scripts': [
        'main = uav_rotor_failure_allocation.cli:main',
        'node = uav_rotor_failure_allocation.ros_node:main']},
)
