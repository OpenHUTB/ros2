from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_obstacle_nn'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'data', 'models']),
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
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
        (
            'share/' + package_name + '/worlds',
            glob('worlds/*.sdf'),
        ),
        (
            'share/' + package_name + '/models',
            glob('models/*.pt'),
        ),
    ],
    install_requires=[
        'setuptools>=30.3.0,<80',
        # Floor matches the numpy shipped with ROS 2 Humble (1.21.x) so
        # installing this package never pulls a newer numpy into
        # ~/.local that would shadow the one ROS 2 was built against.
        'numpy>=1.21,<2.0',
    ],
    entry_points={
        'console_scripts': [
            'main = lidar_obstacle_nn.main:main',
            'inference_node = lidar_obstacle_nn.inference_node:main',
            'collect_data = lidar_obstacle_nn.collect_data:main',
            'generate_synthetic_dataset = lidar_obstacle_nn.generate_synthetic_dataset:main',
            'train_model = lidar_obstacle_nn.train_model:main',
        ],
    },
    zip_safe=True,
    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',
    description=(
        'ROS 2 Humble neural network module that classifies LiDAR '
        'scans into obstacle proximity categories using a PyTorch MLP.'
    ),
    license='Apache-2.0',
)
