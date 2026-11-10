from glob import glob
import os

from setuptools import setup

package_name = "uwb_range_odom_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="07300014",
    maintainer_email="07300014@users.noreply.github.com",
    author="07300014",
    author_email="07300014@users.noreply.github.com",
    description="UWB range and wheel-odometry fusion for planar robot localisation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "uwb_sim = uwb_range_odom_fusion.nodes:main_simulator",
            "uwb_fusion = uwb_range_odom_fusion.nodes:main_fusion",
            "uwb_headless = uwb_range_odom_fusion.headless:main",
        ],
    },
)
