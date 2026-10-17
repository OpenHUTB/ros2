from setuptools import setup
from glob import glob

package_name = 'drone_gazebo_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6lebron23',
    maintainer_email='6lebron23@todo.todo',
    description='无人机Gazebo仿真：四旋翼键盘控制+摄像头神经网络感知',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_teleop = drone_gazebo_sim.teleop_node:main',
            'drone_perception = drone_gazebo_sim.perception_node:main',
            'drone_controller = drone_gazebo_sim.controller_node:main',
            'drone_bridge = drone_gazebo_sim.bridge_node:main',
        ],
    },
)
