from setuptools import setup
import os
from glob import glob

package_name = "mjc_rov_sonar_cnn"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Side-scan sonar sim + CNN seafloor object classification.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim_node = mjc_rov_sonar_cnn.sim_node:main",
            "classify_node = mjc_rov_sonar_cnn.classify_node:main",
        ],
    },
)
