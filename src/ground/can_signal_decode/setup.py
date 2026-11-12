from setuptools import find_packages, setup

package_name = 'can_signal_decode'

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
    description='CAN 报文信号位域解码与 CRC8/滚动计数器校验',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = can_signal_decode.main:main',
            'main_node = can_signal_decode.main_node:main',
        ],
    },
)
