from glob import glob
import os

from setuptools import setup

package_name = "ros2_occupancy_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="LinFengCi",
    maintainer_email="linfengci@users.noreply.github.com",
    description="Log-odds 2-D occupancy grid mapping",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_mapping = ros2_occupancy_mapping.main_mapping:main",
        ],
    },
)
