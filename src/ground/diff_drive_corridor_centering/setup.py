from glob import glob
from setuptools import find_packages, setup

package_name = "diff_drive_corridor_centering"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jiangdong000",
    maintainer_email="jiangdong000@example.com",
    description="Differential-drive corridor centering PD cruise with two-side lidar.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "corridor_scan_sim = diff_drive_corridor_centering.nodes:scan_main",
            "corridor_centering = diff_drive_corridor_centering.nodes:centering_main",
        ],
    },
)
