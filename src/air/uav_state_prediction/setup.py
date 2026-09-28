from glob import glob
from setuptools import setup

setup(name='uav_state_prediction',version='0.1.0',packages=['uav_prediction'],
    data_files=[('share/ament_index/resource_index/packages',['resource/uav_state_prediction']),
                ('share/uav_state_prediction',['package.xml']),
                ('share/uav_state_prediction/launch',glob('launch/*.py')),
                ('share/uav_state_prediction/rviz',glob('rviz/*.rviz'))],
    install_requires=['setuptools'],zip_safe=True,
    description='AirSim multi-horizon neural flight-state prediction and ROS2 visualization',
    maintainer='zijuan xiao',maintainer_email='2535030075@stu.hutb.edu.cn',license='Apache-2.0',
    entry_points={'console_scripts':['state_source=uav_prediction.ros_nodes:source_main',
                                    'state_predictor=uav_prediction.ros_nodes:predictor_main',
                                    'topic_plot=uav_prediction.plot_node:main']})
