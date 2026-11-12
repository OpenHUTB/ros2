from setuptools import find_packages, setup

package_name = 'wheel_speed_observer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huangjiechen',
    maintainer_email='huangjiechen00@163.com',
    description='轮速传感器 + 龙伯格观测器估计传动轴扭角',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = wheel_speed_observer.main:main',
            'main_node = wheel_speed_observer.main_node:main',
        ],
    },
)