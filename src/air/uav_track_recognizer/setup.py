from setuptools import setup

package_name = 'uav_track_recognizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='UAV track recognizer node, main entry: main_uav_track.py',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_track = uav_track_recognizer.main_uav_track:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uav_track.launch.py']),
    ],
)
