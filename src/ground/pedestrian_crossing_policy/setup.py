import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pedestrian_crossing_policy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zheyexia10",
    maintainer_email="zheyexia10@users.noreply.github.com",
    description="TTC-based pedestrian curb-crossing policy and ROS 2 adapter",
    license="MIT",
    entry_points={
        "console_scripts": [
            "crossing_policy_node = pedestrian_crossing_policy.node:main",
        ],
    },
)
