from setuptools import find_packages, setup

package_name = 'tb3_vfh_avoidance'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/avoid.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='黄家欣',
    maintainer_email='326516287+usernamekkyy@users.noreply.github.com',
    description='TurtleBot3 激光雷达 VFH 局部避障（配置空间膨胀 + 谷值搜索）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_node = tb3_vfh_avoidance.avoid_node:main',
        ],
    },
)
