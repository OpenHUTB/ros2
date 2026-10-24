from setuptools import setup

package_name = 'humanoid_zmp_walk_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/resource_index']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='student',
    maintainer_email='2320842128@qq.com',
    description='Humanoid ZMP walking trajectory planner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = humanoid_zmp_walk_planner.main:main',
        ],
    },
)
