from setuptools import find_packages, setup

package_name = 'waypoint_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SYQ427',
    maintainer_email='syq427@github.com',
    description='ROS2 Humble waypoint publisher node for Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_pub = waypoint_publisher.waypoint_pub:main',
        ],
    },
)
