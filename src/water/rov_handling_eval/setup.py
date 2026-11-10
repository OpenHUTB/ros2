# -*- coding: utf-8 -*-
from glob import glob

from setuptools import find_packages, setup

package_name = "rov_handling_eval"

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
    install_requires=["setuptools", "numpy", "mujoco"],
    zip_safe=True,
    maintainer="lxy020208",
    maintainer_email="2334936626@qq.com",
    description="ROV 六自由度手操与抗流能力评测仿真",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_sim = rov_handling_eval.mujoco_sim:main",
            "keyboard_teleop = rov_handling_eval.keyboard_teleop:main",
            "eval_node = rov_handling_eval.eval_node:main",
        ],
    },
)
