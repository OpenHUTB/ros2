from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'mujoco_auto_parking'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'models'),
            glob('models/*.xml') + glob('models/*.npz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description=(
        'MuJoCo differential-drive auto-parking simulation '
        '/ MuJoCo 四轮差速车自动泊车仿真'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = mujoco_auto_parking.simulation_node:main',
            'simulation_node = mujoco_auto_parking.simulation_node:main',
            'keyboard_teleop_node = mujoco_auto_parking.keyboard_teleop_node:main',
            'perception_node = mujoco_auto_parking.perception_node:main',
            'trajectory_control_node = mujoco_auto_parking.trajectory_control_node:main',
            'navigation_node = mujoco_auto_parking.navigation_node:main',
            'end_to_end_node = mujoco_auto_parking.end_to_end_node:main',
        ],
    },
)
