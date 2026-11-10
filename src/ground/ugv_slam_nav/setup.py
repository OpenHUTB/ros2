import os
from glob import glob
from setuptools import setup

package_name = "ugv_slam_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enemy52",
    maintainer_email="student@example.com",
    description="Ground UGV SLAM mapping and navigation (turtlebot3 + slam_toolbox + nav2)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ugv_odom_tf = ugv_slam_nav.odom_tf_node:main",
            "ugv_simple_nav = ugv_slam_nav.simple_nav_node:main",
            "ugv_nav_relay = ugv_slam_nav.nav_relay_node:main",
            "main = ugv_slam_nav.cli:main",
        ],
    },
)
