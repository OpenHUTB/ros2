from setuptools import find_packages, setup

package_name = 'auv_end2end'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liahn',
    maintainer_email='liahn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auv_vision_control = auv_end2end.auv_vision_control:main',
            'auv_obstacle_avoid = auv_end2end.auv_obstacle_avoid:main',
            'auv_vel_ctrl = auv_end2end.auv_vel_ctrl:main',
            'diff_drive = auv_end2end.diff_drive:main'
        ]
    },
) 

