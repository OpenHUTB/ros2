from setuptools import setup

package_name = 'usv_colregs_avoidance'

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
    description='USV COLREGs encounter classification and avoidance decision',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'main = usv_colregs_avoidance.main:main',
        ],
    },
)
