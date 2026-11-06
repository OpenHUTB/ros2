from setuptools import setup

package_name = 'smooth_keyboard_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
        entry_points={
    'console_scripts': [
        'smooth_teleop = smooth_keyboard_teleop.smooth_teleop:main',
        'keyboard_control = smooth_keyboard_teleop.keyboard_control:main',
    ],
},

)
