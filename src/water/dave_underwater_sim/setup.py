"""ROS 2 ament_python 包与命令行入口（包装 DAVE ROS 1 演示脚本）。"""
from glob import glob
from os.path import join

from setuptools import setup

package_name = "dave_underwater_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (join("share", package_name), ["package.xml"]),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name), ["run_demo.sh", "main.sh", "install_dave.sh"]),
        (join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "dave_underwater_sim = dave_underwater_sim.ros_node:main",
        ],
    },
)