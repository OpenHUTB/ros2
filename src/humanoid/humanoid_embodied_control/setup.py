from glob import glob

from setuptools import find_packages, setup

package_name = "humanoid_embodied_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests", "scripts"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*.xml")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Mjm-peter",
    maintainer_email="Mjm-peter@users.noreply.github.com",
    description="人形机器人（具身人）物理仿真、多传感器感知与平衡行走控制",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "humanoid_walk_node = humanoid_embodied_control.nodes.walk_node:main",
        ],
    },
)
