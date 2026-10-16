from setuptools import setup

package_name = 'ultra_avoid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/car.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/car.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 ultrasonic obstacle avoid',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultra_node = ultra_avoid.ultra_node:main',
        ],
    },
)
