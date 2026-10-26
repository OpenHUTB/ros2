"""ROS 2 包安装配置。 / ROS 2 package installation."""
from glob import glob
from setuptools import find_packages, setup

NAME = "mujoco_warehouse_uav_collision_braking"
setup(name=NAME, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + NAME]),
                  ("share/" + NAME, ["package.xml"]),
                  ("share/" + NAME + "/launch", glob("launch/*.py"))],
      install_requires=["setuptools", "numpy", "mujoco", "Pillow"],
      entry_points={"console_scripts": ["main.py = warehouse_braking.main:main",
                                         "sensor_node = warehouse_braking.ros_node:main",
                                         "learned_controller = warehouse_braking.learned_node:main",
                                         "train_model = warehouse_braking.train_model:main",
                                         "evaluate_model = warehouse_braking.evaluate_model:main"]})
