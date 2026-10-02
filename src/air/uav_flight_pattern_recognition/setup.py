from setuptools import setup
from glob import glob

setup(name='uav_flight_pattern_recognition', version='0.1.0',
      py_modules=['pattern_data', 'pattern_model', 'pattern_stream', 'ros_pattern', 'pattern_dashboard', 'pattern_live'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_flight_pattern_recognition']),
                  ('share/uav_flight_pattern_recognition', ['package.xml']),
                  ('share/uav_flight_pattern_recognition/launch', glob('launch/*.py'))],
      install_requires=['setuptools'],
      maintainer='zijuan xiao', maintainer_email='2535030075@stu.hutb.edu.cn',
      description='Causal neural UAV trajectory classification and ROS2 task logs',
      license='Apache-2.0', entry_points={'console_scripts': [
          'pattern_replay=ros_pattern:replay_main',
          'pattern_recognizer=ros_pattern:recognize_main',
          'pattern_dashboard=pattern_dashboard:main',
          'pattern_live=pattern_live:main']})
