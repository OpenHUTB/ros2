from setuptools import find_packages, setup

package_name = 'water'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rov.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/underwater.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='CNN sonar underwater target recognition',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_infer_node = water.sonar_infer_node:main',
        ],
    },
)
