from setuptools import setup

package_name = 'turtle_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/turtle_circle.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huruichen',
    maintainer_email='huruichen@users.noreply.github.com',
    description='Turtlesim circle drawing (ROS 2 port of docs chapter 1)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 仓库约定：模块入口为 main.*
            'main = turtle_motion.circle_node:main',
            'circle_draw = turtle_motion.circle_node:main',
        ],
    },
)
