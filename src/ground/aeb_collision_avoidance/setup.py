from glob import glob
import os

from setuptools import setup

package_name = "aeb_collision_avoidance"

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
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="maintainer@example.com",
    description="TTC-based autonomous emergency braking for ground vehicles",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aeb_node = aeb_collision_avoidance.aeb_node:main",
        ],
    },
)
