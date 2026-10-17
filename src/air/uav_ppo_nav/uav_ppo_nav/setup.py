from setuptools import setup
package_name = 'uav_ppo_nav'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': ['ppo_nav_node = uav_ppo_nav.ppo_nav_node:main']},
)
