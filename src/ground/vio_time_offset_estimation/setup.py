from glob import glob
import os

from setuptools import setup

package_name = "vio_time_offset_estimation"

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
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="07300014",
    maintainer_email="07300014@users.noreply.github.com",
    author="07300014",
    author_email="07300014@users.noreply.github.com",
    description="Visual-inertial time offset estimation by cross-correlation of gyro and image-rate signals.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vio_sim = vio_time_offset_estimation.nodes:main_vio_sim",
            "time_offset_estimator = vio_time_offset_estimation.nodes:main_time_offset_estimator",
        ],
    },
)
