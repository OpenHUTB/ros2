from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=["rov_mujoco_slam_nn_nav"], package_dir={"": "."}))
