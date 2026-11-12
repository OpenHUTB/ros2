from setuptools import setup

package_name = 'service_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch',
        ['launch/move_forward.launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zijuan',
    maintainer_email='2535030075@stu.hutb.edu.cn',
    description='ROS2 service example for distance-based turtlesim control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
    'main = service_demo.main:main',
    'server = service_demo.server:main',
    'client = service_demo.client:main',
],
},
)
