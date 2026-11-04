import os
from glob import glob

from setuptools import setup

package_name = "mujoco_muscle_leg_gait"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "main.py", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB",
    maintainer_email="openhutb@example.com",
    description="肌肉驱动下肢：从单关节到负重踏步的六级实验",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "muscle_leg_gait = mujoco_muscle_leg_gait.cli:main",
        ],
    },
)
