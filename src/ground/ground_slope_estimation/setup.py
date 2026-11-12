from setuptools import setup

package_name = 'ground_slope_estimation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='234208483+gfdklzr@users.noreply.github.com',
    description='从深度点云拟合地面平面并估计坡度角',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slope_sim = ground_slope_estimation.slope_sim:main',
            'slope_report = ground_slope_estimation.slope_report:main',
        ],
    },
)
