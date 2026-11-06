from setuptools import setup

package_name = 'underwater_sonar_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='noreply@openhutb.github.io',
    description='Synthetic underwater sonar mapping and obstacle visualization',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main = underwater_sonar_mapping.main:main',
            'render_scene = underwater_sonar_mapping.scene:main',
        ],
    },
)