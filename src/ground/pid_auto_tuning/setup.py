from setuptools import find_packages, setup

package_name = 'pid_auto_tuning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='差速小车航向 PID 参数自整定与手工试凑对比',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = pid_auto_tuning.main:main',
            'main_node = pid_auto_tuning.main_node:main',
            'main_plot = pid_auto_tuning.plot_view:main',
        ],
    },
)
