from glob import glob

from setuptools import find_packages, setup

package_name = "mujoco_musculoskeletal_arm_reach"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/models", glob("models/*.xml")),
    ],
    install_requires=["setuptools", "numpy", "mujoco", "pyyaml"],
    zip_safe=True,
    maintainer="胡睿晨",
    maintainer_email="huruichen@users.noreply.github.com",
    description="双关节肌肉骨骼手臂到达控制（MuJoCo + ROS 2）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_reach_node = mujoco_musculoskeletal_arm_reach.node:main",
        ],
    },
)
