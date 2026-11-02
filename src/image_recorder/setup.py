from setuptools import setup

package_name = 'image_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='student',
    maintainer_email='3523828193@qq.com', 
    description='ROS 2 Carla image recorder package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = image_recorder.main:main', 
        ],
    },
)
