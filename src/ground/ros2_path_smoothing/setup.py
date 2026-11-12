from glob import glob
import os

from setuptools import setup

package_name = "ros2_path_smoothing"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="LinFengCi",
    maintainer_email="linfengci@users.noreply.github.com",
    description="A* + Catmull-Rom path smoothing",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_smooth = ros2_path_smoothing.main_smooth:main",
        ],
    },
)
