from setuptools import setup

package_name = 'underwater_sonar_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/main.launch.py', 'launch/sonar_vfh.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Underwater sonar mapping with obstacles',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = underwater_sonar_mapping.main:main',
            'sonar_vfh = underwater_sonar_mapping.sonar_vfh_node:main',
        ],
    },
)
