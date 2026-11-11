# -*- coding: utf-8 -*-
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_rotor_failure_env"

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
    description="多旋翼单桨失效后的悬停可行域",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "allocation_node = uav_rotor_failure_env.ros_node:main",
        ],
    },
)
