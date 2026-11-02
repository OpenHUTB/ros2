from setuptools import find_packages, setup

package_name = 'uav_airsim_keyboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/uav_airsim_keyboard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dniuna',
    maintainer_email='dniuna@example.com',
    description='AirSim Blocks 四旋翼无人机键盘遥控（ROS2 Humble）',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_node = uav_airsim_keyboard.main:main',
        ],
    },
)
