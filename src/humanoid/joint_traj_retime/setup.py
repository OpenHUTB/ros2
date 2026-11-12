# -*- coding: utf-8 -*-
from glob import glob

from setuptools import find_packages, setup

package_name = "joint_traj_retime"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="lxy020208",
    maintainer_email="2334936626@qq.com",
    description="关节轨迹的时间最优重定时",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "retime_node = joint_traj_retime.ros_node:main",
        ],
    },
)
