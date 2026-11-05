from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'mujoco_diff_drive_slip_compensation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description=(
        'MuJoCo differential-drive slip perception and trajectory compensation '
        '/ MuJoCo 差速小车轮滑感知与轨迹补偿'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = mujoco_diff_drive_slip_compensation.main:main',
        ],
    },
)
