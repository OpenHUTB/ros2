from setuptools import setup

package_name = 'px4_ros2_opencv_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracking_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='PX4 + ROS2 + OpenCV 无人机视觉目标追踪',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_node = px4_ros2_opencv_tracking.node:main',
        ],
    },
)
