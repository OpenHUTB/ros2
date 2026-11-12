from glob import glob

from setuptools import setup

package_name = 'humanoid_imu_attitude'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='sousa',
    maintainer_email='sousa0913@qq.com',
    description='Compare complementary and Madgwick attitude estimation on a humanoid IMU stream.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_sim = humanoid_imu_attitude.imu_sim:main',
            'attitude_filter = humanoid_imu_attitude.attitude_filter:main',
            'attitude_eval = humanoid_imu_attitude.evaluate:main',
        ],
    },
)
