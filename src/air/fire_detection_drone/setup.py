from setuptools import setup

package_name = 'fire_detection_drone'

setup(
    name=package_name,
    version='1.0.0',
    description='ROS 2 fire detection and autonomous exploration package for UAVs.',
    maintainer='Xiangyuetang91',
    maintainer_email='2865306093@qq.com',
    license='Apache-2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'README.md', 'LICENSE', 'requirements.txt']),
        ('share/' + package_name + '/launch',
         ['launch/fire_detection_drone.launch.py', 'launch/main.launch.py']),
        ('share/' + package_name + '/config',
         ['config/fire_detection.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'main = fire_detection_drone.main:main',
            'fire_detector = fire_detection_drone.fire_detector_node:main',
            'drone_explorer = fire_detection_drone.drone_explorer_node:main',
        ],
    },
)
