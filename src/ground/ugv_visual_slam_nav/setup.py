from glob import glob

from setuptools import setup

package_name = 'ugv_visual_slam_nav'

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
    description='地面无人车视觉 SLAM 建图与自主导航（EKF-SLAM + 栅格 + A* + 端到端 CNN）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = ugv_visual_slam_nav.main:main',
            'sim = ugv_visual_slam_nav.sim_node:main',
            'slam = ugv_visual_slam_nav.slam_node:main',
            'nav = ugv_visual_slam_nav.nav_node:main',
            'e2e = ugv_visual_slam_nav.e2e_node:main',
        ],
    },
)
