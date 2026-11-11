from setuptools import find_packages, setup

package_name = 'humanoid_visual_servo_grasp'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/models',
            ['models/humanoid_visual_servo_grasp.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenHUTB',
    maintainer_email='openhutb@example.com',
    description='人形机器人视觉伺服抓取：单臂 + 两指夹具在 MuJoCo 中感知桌面目标物并完成抓取',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = humanoid_visual_servo_grasp.mujoco_sim_node:main',
        ],
    },
)
