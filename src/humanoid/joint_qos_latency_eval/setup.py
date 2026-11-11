from glob import glob

from setuptools import setup

package_name = 'joint_qos_latency_eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sousa',
    maintainer_email='235034439+Sousa-cc@users.noreply.github.com',
    description='关节指令的 QoS 与时延影响评估',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_qos_pub = joint_qos_latency_eval.joint_pub:main',
            'joint_qos_sub = joint_qos_latency_eval.joint_sub:main',
            'qos_summarize = joint_qos_latency_eval.summarize:main',
        ],
    },
)
