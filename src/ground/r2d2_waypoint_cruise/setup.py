from setuptools import find_packages, setup
import os
from glob import glob

package_name = "r2d2_waypoint_cruise"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xjm666god",
    maintainer_email="noreply@example.com",
    description="R2D2 差速小车航点巡航仿真（PyBullet），P控制器沿方形路径循环巡逻",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "r2d2_waypoint_cruise = r2d2_waypoint_cruise.cruise:main",
        ],
    },
)
