from glob import glob

from setuptools import setup

package_name = 'train_ato_stop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='李英杰',
    maintainer_email='327110089+lzxbaba@users.noreply.github.com',
    description='列车 ATO 自动运行与站台精确停车仿真',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_ato = train_ato_stop.main:main',
            'train_ato_node = train_ato_stop.ros_node:main',
        ],
    },
)
