import os
from glob import glob
from setuptools import setup

package_name = 'uav_key_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
      data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sousa',
    maintainer_email='sousa0913@qq.com',
    description='UAV keyboard velocity control node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'key_ctrl = uav_key_ctrl.keyboard_uav:main',
        ],
    },
)
