from glob import glob
import os

from setuptools import setup

package_name = 'privacy_mask'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # resource 下的 png 与模型权重要一起安装到 share，否则节点找不到文件
        (os.path.join('share', package_name, 'resource'),
         glob('resource/*.png')),
        (os.path.join('share', package_name, 'resource'),
         glob('resource/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='刘琪',
    maintainer_email='2754586324@qq.com',
    description='基于 YOLOv8 的行人隐私脱敏 ROS 2 节点',
    license='AGPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 可执行名与 launch 文件中的 executable 保持一致
            'privacy_mask_main = privacy_mask.main:main',
        ],
    },
)
