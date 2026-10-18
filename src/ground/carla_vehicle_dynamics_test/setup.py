from glob import glob
import os

from setuptools import setup

package_name = 'carla_vehicle_dynamics_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'main.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='openhutb@example.com',
    description='CARLA 地面载具动力性测试：0→v 加速时间、制动距离、稳态转弯半径',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamics_test_node = carla_vehicle_dynamics_test.dynamics_node:main',
        ],
    },
)
