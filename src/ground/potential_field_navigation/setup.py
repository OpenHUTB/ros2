from glob import glob
from os import path

from setuptools import find_packages, setup

package_name = "potential_field_navigation"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        (path.join("share", package_name), ["package.xml"]),
        (path.join("share", package_name, "launch"),
         glob(path.join("launch", "*.launch.py"))),
        (path.join("share", "ament_index", "resource_index", "packages"),
         [path.join("resource", package_name)]),
    ],
    install_requires=["pygame>=2.0"],
    zip_safe=True,
    maintainer="wqs159",
    maintainer_email="799628765@qq.com",
    description="人工势场法导航 2D 仿真与位姿/轨迹发布节点",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"{package_name} = {package_name}.main:main",
            f"potential_field_node = {package_name}.node:main",
        ],
    },
)
