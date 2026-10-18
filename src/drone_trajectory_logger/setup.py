from setuptools import setup

package_name = 'drone_trajectory_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='3523828193@qq.com',
    description='Drone flight status recognition and trajectory plotting module',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = drone_trajectory_logger.main:main',
            'analyzer = drone_trajectory_logger.trajectory_analyzer:main',
        ],
    },
)
