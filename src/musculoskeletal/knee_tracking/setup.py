from glob import glob

from setuptools import find_packages, setup

package_name = "mujoco_musculoskeletal_knee_tracking"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/assets", glob("assets/*.xml")),
    ],
    install_requires=["setuptools", "numpy", "mujoco", "pyyaml"],
    zip_safe=True,
    maintainer="胡睿晨",
    maintainer_email="huruichen@users.noreply.github.com",
    description="单关节膝关节肌肉骨骼仿真与轨迹跟踪（MuJoCo + ROS 2）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "knee_tracking_node = mujoco_musculoskeletal_knee_tracking.node:main",
        ],
    },
)
