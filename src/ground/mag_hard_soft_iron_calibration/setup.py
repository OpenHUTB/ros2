from glob import glob
import os

from setuptools import setup

package_name = "mag_hard_soft_iron_calibration"

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
    description="Magnetometer hard/soft-iron calibration and tilt-compensated heading fusion.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mag_sim = mag_hard_soft_iron_calibration.nodes:main_mag_sim",
            "mag_calibrator = mag_hard_soft_iron_calibration.nodes:main_mag_calibrator",
        ],
    },
)
