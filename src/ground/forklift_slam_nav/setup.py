from setuptools import setup
from glob import glob

package_name = 'forklift_slam_nav'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml')),
                  ('share/' + package_name + '/worlds', glob('worlds/*.xml'))],
      install_requires=['setuptools', 'numpy>=1.24', 'mujoco==3.12.0'], zip_safe=False,
      tests_require=['pytest'],
      entry_points={'console_scripts': ['main = forklift_slam_nav.main:main', 'mujoco_robot = forklift_slam_nav.simulator:main']})
