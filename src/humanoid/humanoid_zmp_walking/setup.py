from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_zmp_walking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sousa',
    maintainer_email='your_email@todo.todo',
    description='A lightweight ZMP based bipedal walking baseline',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zmp_generator = humanoid_zmp_walking.zmp_generator:main',
            'play = humanoid_zmp_walking.play:main',
        ],
    },
)
