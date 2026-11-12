from glob import glob
import os

from setuptools import setup

package_name = "ros2_dynamic_obstacle_ttc"

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
    description="2-D dynamic obstacle tracking and TTC warning",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_ttc = ros2_dynamic_obstacle_ttc.main_ttc:main",
        ],
    },
)

