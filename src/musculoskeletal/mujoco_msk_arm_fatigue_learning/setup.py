from glob import glob

from setuptools import find_packages, setup


package_name = 'mujoco_msk_arm_fatigue_learning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',
    description='Fatigue learning control for a musculoskeletal arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'fatigue_learning_node = '
                'mujoco_msk_arm_fatigue_learning.ros_node:main'
            ),
        ],
    },
)
