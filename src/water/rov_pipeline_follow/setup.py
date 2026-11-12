from glob import glob

from setuptools import setup

package_name = 'rov_pipeline_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 预训练权重：节点用 ament share 目录定位，install 后不必关心 CWD
        ('share/' + package_name + '/models', glob('models/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6lebron23',
    maintainer_email='3248146937@qq.com',
    description='水下 ROV 海底管道往返巡检（广角前视相机感知 + EKF-SLAM + 管道跟踪 + 端到端 CNN）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = rov_pipeline_follow.main:main',
            'sim = rov_pipeline_follow.sim_node:main',
            'slam = rov_pipeline_follow.slam_node:main',
            'control = rov_pipeline_follow.control_node:main',
            'e2e = rov_pipeline_follow.e2e_node:main',
        ],
    },
)
