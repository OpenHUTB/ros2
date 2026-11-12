from glob import glob
import os

from setuptools import setup

package_name = "mpc_path_tracking"

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
    description="Linear MPC path tracking with box-constrained QP versus pure pursuit.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mpc_tracker = mpc_path_tracking.nodes:main_mpc_tracker",
            "mpc_reference = mpc_path_tracking.nodes:main_mpc_reference",
        ],
    },
)
