from setuptools import setup
import os
from glob import glob

package_name = "uav_airsim_depth_obstacle_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models_trained"), glob("models_trained/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WZJ6666wudu",
    maintainer_email="WZJ6666wudu@users.noreply.github.com",
    description="AirSim depth-based neural obstacle avoidance",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "depth_avoid_node = uav_airsim_depth_obstacle_avoidance.node:main",
        ],
    },
)
