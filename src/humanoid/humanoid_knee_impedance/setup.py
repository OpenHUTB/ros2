from glob import glob

from setuptools import setup

package_name = 'humanoid_knee_impedance'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools', 'matplotlib'],
    zip_safe=True,
    maintainer='sousa',
    maintainer_email='sousa0913@qq.com',
    description='Knee joint impedance gains tuning: stiffness and damping effects on a position step response.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'knee_sim = humanoid_knee_impedance.knee_sim:main',
            'knee_report = humanoid_knee_impedance.report:main',
        ],
    },
)
