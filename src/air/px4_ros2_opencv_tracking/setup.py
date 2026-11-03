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
    maintainer='Chenjunhua',
    maintainer_email='chenjunhua@users.noreply.github.com',
    description='PX4 + ROS2 + OpenCV 无人机视觉目标追踪',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pub = px4_ros2_opencv_tracking.image_pub:main',
            'color_tracker = px4_ros2_opencv_tracking.color_tracker:main',
            'offboard_control = px4_ros2_opencv_tracking.offboard_control:main',
        ],
    },
)
