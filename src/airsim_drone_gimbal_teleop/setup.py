from setuptools import setup

package_name = 'airsim_drone_gimbal_teleop'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LJH0786',
    maintainer_email='325837027+LJH0786@users.noreply.github.com',
    description='AirSim 无人机键盘遥控与机载云台相机预览控制模块',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_gimbal_teleop = airsim_drone_gimbal_teleop.main:main',
        ],
    },
)
