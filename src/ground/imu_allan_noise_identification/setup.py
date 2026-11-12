from glob import glob
import os

from setuptools import setup

package_name = "imu_allan_noise_identification"

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
    description="Allan-variance IMU noise identification and filter tuning for a ground robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "imu_sim = imu_allan_noise_identification.nodes:main_imu_sim",
            "allan_identifier = imu_allan_noise_identification.nodes:main_allan_identifier",
        ],
    },
)
