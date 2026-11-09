from glob import glob
from setuptools import setup
setup(name='payload_mass_estimation',version='0.1.0',packages=['payload_mass_estimation'],
      data_files=[('share/ament_index/resource_index/packages',['resource/payload_mass_estimation']),
                  ('share/payload_mass_estimation',['package.xml']),('share/payload_mass_estimation/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],zip_safe=True,maintainer='zijuan xiao',maintainer_email='2535030075@stu.hutb.edu.cn',
      description='Neural payload mass estimation from calibrated thrust and vertical flight',license='MIT',
      entry_points={'console_scripts':['payload_replay=payload_mass_estimation.ros_nodes:replay','payload_estimator=payload_mass_estimation.ros_nodes:estimate']})
