from setuptools import setup
from glob import glob

package_name = 'rl_algorithm_compare'

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
    description='在 MuJoCo CartPole 上用纯NumPy神经网络实现RL算法与网络结构对比（DQN/REINFORCE、单双层MLP）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'train = rl_algorithm_compare.train:main',
            'evaluate = rl_algorithm_compare.evaluate:main',
            'compare = rl_algorithm_compare.compare:main',
            'render_gif = rl_algorithm_compare.render_gif:main',
            'main = rl_algorithm_compare.main:main',
        ],
    },
)
