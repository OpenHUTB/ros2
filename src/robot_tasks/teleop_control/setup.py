from setuptools import find_packages, setup

package_name = 'teleop_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='1woshichenwenguang1',
    maintainer_email='2507417911@qq.com',
    description='ROS 2 Keyboard Teleop Control Module',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_teleop = teleop_control.main_teleop:main',
        ],
    },
)
