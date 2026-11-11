from glob import glob
import os

from setuptools import setup

package_name = "ros2_dwa_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LinFengCi",
    maintainer_email="linfengci@users.noreply.github.com",
    description="Gazebo 11 + TurtleBot3 3-D validation for the A*+DWA planner",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "verify_nav = ros2_dwa_gazebo.verify_nav:main",
        ],
    },
)
