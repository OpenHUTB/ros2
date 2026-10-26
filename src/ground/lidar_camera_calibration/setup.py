from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_camera_calibration'

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
            'main = lidar_camera_calibration.main:main',
            'run_calibration = lidar_camera_calibration.run_calibration:main',
            'sweep_noise = lidar_camera_calibration.sweep_noise:main',
            'sweep_views = lidar_camera_calibration.sweep_views:main',
            'project_cloud = lidar_camera_calibration.project_cloud:main',
            'calibration_node = lidar_camera_calibration.calibration_node:main',
        ],
    },
    zip_safe=True,
    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',
    description=(
        'ROS 2 Humble module for LiDAR-camera extrinsic calibration: '
        'synthetic multi-view chessboard scene, PnP solving and '
        'quantitative error analysis against ground truth.'
    ),
    license='Apache-2.0',
)
