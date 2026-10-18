import os
from glob import glob

from setuptools import find_packages, setup

package_name = "driving_study_analyzer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "data"), glob("data/*.csv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zheyexia10",
    maintainer_email="zheyexia10@users.noreply.github.com",
    description="Offline analyzer for DReyeVR driving-study CSV sessions",
    license="MIT",
    entry_points={
        "console_scripts": [
            "driving_study_analyzer_node = driving_study_analyzer.node:main",
        ],
    },
)
