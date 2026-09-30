from setuptools import setup
from glob import glob

setup(name='uav_telemetry_diagnostics', version='0.1.0',
      py_modules=['telemetry_data', 'train_evaluate', 'stream_detector', 'ros_diagnostics', 'dashboard',
                  'live_source', 'collect_holdout', 'consistency'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_telemetry_diagnostics']),
                  ('share/uav_telemetry_diagnostics', ['package.xml']),
                  ('share/uav_telemetry_diagnostics/launch', glob('launch/*.py'))],
      install_requires=['setuptools'],
      maintainer='zijuan xiao', maintainer_email='2535030075@stu.hutb.edu.cn',
      description='Neural UAV telemetry anomaly detection with ROS2 diagnostics',
      license='Apache-2.0', entry_points={'console_scripts': [
          'telemetry_replay=ros_diagnostics:replay_main',
          'telemetry_diagnostics=ros_diagnostics:diagnostics_main',
          'telemetry_live=live_source:main',
          'telemetry_dashboard=dashboard:main']})
