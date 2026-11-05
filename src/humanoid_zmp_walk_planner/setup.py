from setuptools import setup

package_name = 'humanoid_zmp_walk_planner'

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
    maintainer_email='2320842128@qq.com',
    description='Simplified bipedal gait animation demo for humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_zmp_walk_planner = humanoid_zmp_walk_planner.main:main',
        ],
    },
)
