from glob import glob
import os

from setuptools import setup

package_name = "pix2cmd_end2end_driving"

setup(
    name=package_name,
    version="0.1.0",
    packages=["pix2cmd"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="maintainer@example.com",
    description="End-to-end driving with a numpy CNN",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["pix2cmd_node = pix2cmd.node:main"]},
)