import os
from glob import glob
from setuptools import find_packages, setup

package_name = "pedestrian_group_crossing"
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
    description="Group-aware pedestrian crossing decision support",
    license="MIT",
    entry_points={"console_scripts": ["group_crossing_node = pedestrian_group_crossing.node:main"]},
)
