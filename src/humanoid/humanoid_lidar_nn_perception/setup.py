from setuptools import find_packages, setup
import os
from glob import glob

package_name = "humanoid_lidar_nn_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models", "g1"),
            ["models/g1/scene_g1.xml", "models/g1/scene.xml"]),
        (os.path.join("share", package_name, "models", "g1", "unitree_g1"),
            glob("models/g1/unitree_g1/*.xml")),
        (os.path.join("share", package_name, "models", "g1", "unitree_g1", "assets"),
            glob("models/g1/unitree_g1/assets/*.STL")),
        (os.path.join("share", package_name, "models", "g1", "assets"),
            glob("models/g1/assets/*")),
        (os.path.join("share", package_name, "weights"),
            glob("weights/*.onnx") + glob("weights/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="maintainer@example.com",
    description="G1 2D LiDAR CNN perception and neural obstacle avoidance",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = humanoid_lidar_nn_perception.sim_node:main",
            "perception_node = humanoid_lidar_nn_perception.perception_node:main",
            "avoidance_node = humanoid_lidar_nn_perception.avoidance_node:main",
            "keyboard_teleop = humanoid_lidar_nn_perception.keyboard_teleop:main",
        ],
    },
)
