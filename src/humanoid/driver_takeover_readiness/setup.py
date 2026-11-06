import os
from glob import glob
from setuptools import find_packages, setup

package_name = "driver_takeover_readiness"
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
    description="Explainable driver takeover-readiness scoring",
    license="MIT",
    entry_points={"console_scripts": ["takeover_readiness_node = driver_takeover_readiness.node:main"]},
)
