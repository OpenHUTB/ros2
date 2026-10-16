from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['airsim_rl_planner_ros1'], package_dir={'': 'src'}))
