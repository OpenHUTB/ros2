from glob import glob
import os

from setuptools import setup

package_name = "uav_mujoco_depth_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=["uav_nav"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="maintainer@example.com",
    description="UAV depth-camera neural obstacle avoidance in MuJoCo",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["uav_depth_avoidance_node = uav_nav.node:main"]},
)
