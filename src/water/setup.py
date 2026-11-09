from setuptools import setup

package_name = 'water'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", ["launch/water.launch.py"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # ========= 这里改成你的信息 =========
    maintainer='Xavier-101666',
    maintainer_email='2754586324@qq.com',
    # ===================================
    description='Sonar CNN underwater target recognition ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_infer_node = water.sonar_infer_node:main',
        ],
    },
)
