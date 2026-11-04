from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["humanoid_mujoco_slam_nn_nav"],
    package_dir={"": "."},
)

setup(**d)
