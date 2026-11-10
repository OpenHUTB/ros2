from setuptools import setup

package_name = 'car_pid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a12345',
    maintainer_email='a12345@example.com',
    description='PID control for car',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid_control_node = car_pid_control.pid_control_node:main',
        ],
    },
)
