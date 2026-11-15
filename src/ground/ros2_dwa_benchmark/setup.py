from glob import glob
import os

from setuptools import setup

package_name = "ros2_dwa_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LinFengCi",
    maintainer_email="linfengci@users.noreply.github.com",
    description="Randomized stress benchmark and comfort evaluation for the A*+DWA planner",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_benchmark = ros2_dwa_benchmark.main_benchmark:main",
        ],
    },
)
