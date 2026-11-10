import glob
import os

from setuptools import setup

package_name = 'water'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/rov.launch.py']),
    # 世界文件必须安装，否则 launch 里 get_package_share_directory('water')/worlds 找不到
    ('share/' + package_name + '/worlds', ['worlds/underwater.world']),
]

# 训练产出的权重与类别表：存在才安装，避免权重还没训练时 colcon build 失败
weight_files = (glob.glob(os.path.join('weights', '*.pth'))
                + glob.glob(os.path.join('weights', '*.json')))
if weight_files:
    data_files.append(('share/' + package_name + '/weights', weight_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xavier-101666',
    maintainer_email='2754586324@qq.com',
    description='ROS2 package for underwater ROV sonar image classification (CNN)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_infer_node = water.sonar_infer_node:main',
            'train_sonar = water.train_sonar:main',
            'capture_dataset = water.capture_dataset:main',
        ],
    },
)
