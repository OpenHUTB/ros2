from glob import glob
from setuptools import setup
setup(name='landing_assessment',version='0.1.0',packages=['landing_assessment'],
      data_files=[('share/ament_index/resource_index/packages',['resource/landing_assessment']),
                  ('share/landing_assessment',['package.xml']),('share/landing_assessment/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],zip_safe=True,maintainer='zijuan xiao',maintainer_email='2535030075@stu.hutb.edu.cn',
      description='Causal landing-phase and contact-load assessment',license='MIT',
      entry_points={'console_scripts':['landing_replay=landing_assessment.ros_nodes:replay_main','landing_assessment_node=landing_assessment.ros_nodes:assessment_main']})
