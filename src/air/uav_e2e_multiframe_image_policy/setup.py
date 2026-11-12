import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_e2e_multiframe_image_policy"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "main.py"]),
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools", "numpy", "pygame"],
    zip_safe=True,
    maintainer="wqs159",
    maintainer_email="799628765@qq.com",
    description="多帧图像端到端策略仿真",
    license="MIT",
    entry_points={
        "console_scripts": [
            "e2e_node = uav_e2e_multiframe_image_policy.e2e_node:main",
        ],
    },
)
