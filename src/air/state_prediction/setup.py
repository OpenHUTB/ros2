from glob import glob
from setuptools import setup

setup(name='state_prediction',version='0.1.0',packages=['prediction'],
    data_files=[('share/ament_index/resource_index/packages',['resource/state_prediction']),
                ('share/state_prediction',['package.xml']),
                ('share/state_prediction/launch',glob('launch/*.py')),
                ('share/state_prediction/rviz',glob('rviz/*.rviz'))],
    install_requires=['setuptools'],zip_safe=True,
    description='AirSim multi-horizon neural flight-state prediction and ROS2 visualization',
    maintainer='zijuan xiao',maintainer_email='2535030075@stu.hutb.edu.cn',license='Apache-2.0',
    entry_points={'console_scripts':['state_source=prediction.ros_nodes:source_main',
                                    'state_predictor=prediction.ros_nodes:predictor_main',
                                    'topic_plot=prediction.plot_node:main']})
