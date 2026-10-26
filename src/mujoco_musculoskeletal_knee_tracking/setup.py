import glob
import os

from setuptools import setup

package_name = "mujoco_musculoskeletal_knee_tracking"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
        (os.path.join("share", package_name, "model"),
         ["mujoco_musculoskeletal_knee_tracking/model/muscle_knee.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="your_email@example.com",
    description="单关节膝关节肌肉骨骼仿真与轨迹跟踪控制 (MuJoCo)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_musculoskeletal_knee_tracking_node = "
            "mujoco_musculoskeletal_knee_tracking.main:main",
        ],
    },
)
