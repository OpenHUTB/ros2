from glob import glob
import os

from setuptools import setup

package_name = 'tf_health_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='jiaohua745-blip@users.noreply.github.com',
    description='TF 链路健康监视节点：断链、时间戳回退、多 parent 与频率不足检测',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tf_health_monitor = '
            'tf_health_monitor.tf_health_monitor_node:main',
            'tf_demo_publisher = '
            'tf_health_monitor.tf_demo_publisher:main',
        ],
    },
)
