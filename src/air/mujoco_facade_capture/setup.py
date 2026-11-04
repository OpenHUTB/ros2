"""ROS 2 ament_python 包与命令行入口。"""

from glob import glob
from os.path import join

from setuptools import setup

package_name = "mujoco_facade_capture"

setup(
    name=package_name,
    version="0.1.0",
    packages=["facade"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (join("share", package_name), ["package.xml"]),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name, "models"), glob("models/*.json")),
    ],
    install_requires=["setuptools", "numpy", "Pillow", "mujoco==3.12.0"],
    tests_require=["pytest"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "mission = facade.ros_node:main",
        ],
    },
)
