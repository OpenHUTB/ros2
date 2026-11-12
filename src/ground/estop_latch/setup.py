from glob import glob
import os

from setuptools import setup

package_name = 'estop_latch'

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
    maintainer_email='234208483+gfdklzr@users.noreply.github.com',
    description='急停闩锁节点：锁住急停，必须人工复位才放行',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'estop_latch = estop_latch.estop_latch_node:main',
        ],
    },
)
