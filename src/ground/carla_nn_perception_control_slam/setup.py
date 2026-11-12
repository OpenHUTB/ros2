from setuptools import setup

package_name = 'carla_nn_perception_control_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/task1.launch.py',
            'launch/task2.launch.py',
            'launch/task3.launch.py',
            'launch/task4.launch.py',
            'launch/all.launch.py',
        ]),
        ('share/' + package_name + '/models', ['models/pilotnet.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yu071110',
    maintainer_email='yu071110@users.noreply.github.com',
    description='Neural-network perception, control and SLAM for CARLA vehicles',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = ' + package_name + '.main:main',
        ],
    },
)
