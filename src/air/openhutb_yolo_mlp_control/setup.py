from glob import glob
from setuptools import find_packages, setup

package_name = 'openhutb_yolo_mlp_control'

setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
        (
            'share/' + package_name + '/models',
            glob('models/*.pth')
        ),
    ],

    install_requires=[
        'setuptools>=30.3.0,<80',
        'numpy>=1.26,<2.0',
        'msgpack>=1.0',
    ],

    zip_safe=True,

    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',

    description=(
        'OpenHUTB AirSim ROS 2 bridge, '
        'YOLO perception, and MLP trajectory control.'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [

            'airsim_bridge = '
            'openhutb_yolo_mlp_control.'
            'airsim_bridge_node:main',

            'yolo_detector = '
            'openhutb_yolo_mlp_control.'
            'yolo_node:main',

            'trajectory_target = '
            'openhutb_yolo_mlp_control.'
            'trajectory_target_node:main',

            'mlp_controller = '
            'openhutb_yolo_mlp_control.'
            'mlp_control_node:main',

            'generate_training_data = '
            'openhutb_yolo_mlp_control.'
            'generate_training_data:main',

            'train_mlp = '
            'openhutb_yolo_mlp_control.'
            'train_mlp:main',

            'evaluate_trajectories = '
            'openhutb_yolo_mlp_control.'
            'evaluate_trajectories:main',
        ],
    },
)
