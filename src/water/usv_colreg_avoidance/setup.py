from glob import glob

from setuptools import setup

package_name = 'usv_colreg_avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='廖佳杰',
    maintainer_email='324033891+jiaohua745-blip@users.noreply.github.com',
    description='无人水面艇 COLREG 会遇避碰仿真与避让决策',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_colreg = usv_colreg_avoidance.main:main',
            'usv_colreg_node = usv_colreg_avoidance.ros_node:main',
        ],
    },
)
