from glob import glob
from setuptools import find_packages, setup

package_name = 'uav_spiral_search'

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
            ['package.xml', 'main.sh']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=[
        'setuptools>=30.3.0,<80',
        'numpy>=1.26,<2.0',
        'opencv-python>=4.5',
    ],

    zip_safe=True,

    maintainer='OpenHUTB contributor',
    maintainer_email='maintainer@example.com',

    description=(
        'Archimedean-spiral area search with proportional tracking '
        'and HSV target detection for AirSim.'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'spiral_search = '
            'uav_spiral_search.spiral_search_node:main',
        ],
    },
)
