from glob import glob
import os

from setuptools import setup

package_name = 'multi_agent_hunting'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), ['scenario.json']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*'))),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='234208483+gfdklzr@users.noreply.github.com',
    description='多 AUV 协同围捕示例：拍卖算法 + 水声通信模拟',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'multi_agent_hunting = multi_agent_hunting.main:main',
        ],
    },
)
