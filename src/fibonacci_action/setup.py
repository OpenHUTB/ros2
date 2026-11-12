from setuptools import find_packages, setup

package_name = 'fibonacci_action'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', ['action/Fibonacci.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yyyyy2026',
    maintainer_email='yyyyy2026@users.noreply.github.com',
    description='ROS 2 Action 示例：斐波那契数列计算的服务端与客户端',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'server = fibonacci_action.server:main',
            'client = fibonacci_action.client:main',
        ],
    },
)
