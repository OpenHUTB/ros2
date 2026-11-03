from setuptools import setup

package_name = 'mujoco_traffic_light_stop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB Student',
    maintainer_email='openhutb_course@example.com',
    description='MuJoCo 红绿灯启停小车仿真',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_stop = mujoco_traffic_light_stop.main:main',
        ],
    },
)
