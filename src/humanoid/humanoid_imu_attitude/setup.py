from glob import glob

from setuptools import setup

package_name = 'humanoid_landing_impedance'

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
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='sousa',
    maintainer_email='sousa0913@qq.com',
    description='Compare knee impedance control and stiff position hold on bipedal landing impact.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landing_sim = humanoid_landing_impedance.landing_sim:main',
            'landing_evaluate = humanoid_landing_impedance.evaluate:main',
            'landing_report = humanoid_landing_impedance.report:main',
        ],
    },
)
