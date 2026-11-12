from setuptools import setup

package_name = 'trailer_jackknife_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huruichen',
    maintainer_email='huruichen@users.noreply.github.com',
    description='Trailer jackknife risk prediction and reverse speed guard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = trailer_jackknife_guard.main:main',
        ],
    },
)
