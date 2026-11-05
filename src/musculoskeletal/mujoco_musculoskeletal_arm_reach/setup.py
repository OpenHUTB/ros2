from glob import glob

from setuptools import find_packages, setup


package_name = 'mujoco_musculoskeletal_arm_reach'

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
        ('share/' + package_name + '/models', glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',
    description='Two-joint musculoskeletal arm reaching with Hill muscles',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_sim_node = mujoco_musculoskeletal_arm_reach.ros_node:main',
        ],
    },
)
