from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_imu_time_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=[
        'setuptools>=30.3.0,<80',
        # 与 ROS 2 Humble 自带 numpy (1.21.x) 保持一致，避免 --user 安装
        # 更高版本遮蔽系统 numpy。
        'numpy>=1.21,<2.0',
    ],
    entry_points={
        'console_scripts': [
            'main = camera_imu_time_calibration.main:main',
            'sweep_offset = camera_imu_time_calibration.sweep_offset:main',
            'sweep_rate = camera_imu_time_calibration.sweep_rate:main',
            'sweep_noise = camera_imu_time_calibration.sweep_noise:main',
            'compare_attitude = camera_imu_time_calibration.compare_attitude:main',
            'estimator_node = camera_imu_time_calibration.estimator_node:main',
            'synthetic_source = camera_imu_time_calibration.synthetic_source_node:main',
        ],
    },
    zip_safe=True,
    maintainer='jiaohua745-blip',
    maintainer_email='jiaohua745-blip@users.noreply.github.com',
    description=(
        'ROS 2 Humble module for camera-IMU temporal calibration: '
        'estimates the constant time offset between camera-derived angular '
        'rate and IMU gyro by normalized cross-correlation on synthetic data '
        'with known ground truth, and quantifies the resulting attitude error.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
)
