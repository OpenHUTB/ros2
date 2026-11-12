# -*- coding: utf-8 -*-
from glob import glob

from setuptools import find_packages, setup

package_name = "muscle_synergy_lab"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt", "main.py"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/tools", glob("tools/*.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="lxy020208",
    maintainer_email="2334936626@qq.com",
    description="肌肉协同降维（NMF）与恢复质量评估",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "synergy_node = muscle_synergy_lab.ros_node:main",
        ],
    },
)
