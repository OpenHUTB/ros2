from glob import glob
import os

from setuptools import setup

package_name = "rov_thruster_allocation"

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
    description="Thruster configuration matrix and saturation-aware wrench allocation for an ROV.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "thruster_sim = rov_thruster_allocation.nodes:main_thruster_sim",
            "thruster_allocator = rov_thruster_allocation.nodes:main_thruster_allocator",
        ],
    },
)
