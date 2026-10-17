from setuptools import find_packages, setup

package_name = 'auv_end2end'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch启动文件
        ('share/' + package_name + '/launch', [
            'launch/auv_rect_cruise.launch.py',
            'launch/auv_obstacle_avoid.launch.py'
        ]),
        # 安装world仿真场景文件
        ('share/' + package_name + '/worlds', [
            'worlds/underwater.world'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liahn',
    maintainer_email='liahn@todo.todo',
    description='AUV水下机器人仿真功能包，包含视觉避障与矩形巡航功能',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auv_vision_control = auv_end2end.auv_vision_control:main',
            'auv_obstacle_avoid = auv_end2end.auv_obstacle_avoid:main',
            'auv_vel_ctrl = auv_end2end.auv_vel_ctrl:main',
            'diff_drive = auv_end2end.diff_drive:main',
            # 新增：注册矩形巡航节点入口
            'auv_rect_cruise = auv_end2end.auv_rect_cruise:main',
        ],
    },
)

