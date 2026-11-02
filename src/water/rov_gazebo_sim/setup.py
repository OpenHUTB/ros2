from setuptools import setup
from glob import glob

package_name = 'rov_gazebo_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6lebron23',
    maintainer_email='6lebron23@todo.todo',
    description='水下机器人ROV Gazebo仿真：键盘控制+摄像头目标追踪',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov_teleop = rov_gazebo_sim.teleop_node:main',
            'rov_perception = rov_gazebo_sim.perception_node:main',
            'rov_controller = rov_gazebo_sim.controller_node:main',
            'rov_bridge = rov_gazebo_sim.bridge_node:main',
        ],
    },
)
