from setuptools import setup

package_name = 'uav_mission_safety_manager'
setup(
    name=package_name, version='0.1.0',
    description='ROS 2 UAV mission manager with waypoint queues and asynchronous safety state machine.',
    maintainer='LuLuforeverx2',
    maintainer_email='1357170941@qq.com',
    license='Apache-2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE', 'requirements.txt']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
        ('share/' + package_name + '/config', ['config/mission.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    tests_require=['pytest'],
    entry_points={'console_scripts': ['mission_manager = uav_mission_safety_manager.main:main']},
)
