from setuptools import setup
import os
from glob import glob

package_name = 'drone_analyzer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 注册包资源
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件 (这一步极其重要，解决审核员说的缺 launch 问题)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='Drone state recognition node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这里映射你的节点启动命令，请确保你的 py 文件名一致
            'analyzer_node = drone_analyzer.analyzer_node:main',
        ],
    },
)
