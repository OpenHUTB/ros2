from setuptools import setup
from glob import glob

package_name = 'rl_dqn_improvement'

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
    description='DQN 及其改进算法（Double/Dueling DQN）在 MuJoCo CartPole 上的神经网络控制对比',
    license='MIT',
    entry_points={
        'console_scripts': [
            'train = rl_dqn_improvement.train:main',
            'evaluate = rl_dqn_improvement.evaluate:main',
            'compare = rl_dqn_improvement.compare:main',
            'render_gif = rl_dqn_improvement.render_gif:main',
            'main = rl_dqn_improvement.main:main',
        ],
    },
)
