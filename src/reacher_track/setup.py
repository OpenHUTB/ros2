from setuptools import setup
from glob import glob

package_name = 'reacher_track'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6lebron23',
    maintainer_email='3248146937@qq.com',
    description='MuJoCo Reacher 机械臂目标跟踪：感知(关节编码器+目标检测) -> MLP 神经网络 -> 闭环扭矩控制',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = reacher_track.main:main',
        ],
    },
)
