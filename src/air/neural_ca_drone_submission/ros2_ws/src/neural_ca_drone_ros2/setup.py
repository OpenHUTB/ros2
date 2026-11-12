from glob import glob

from setuptools import setup

package_name = "neural_ca_drone_ros2"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("lib/" + package_name, ["scripts/main.py", "scripts/sim_bridge.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chen Kangyi",
    maintainer_email="chenkangyi@example.com",
    description="Neural collision avoidance stack for a drone (ROS 2).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = neural_ca_drone_ros2.main_node:main",
            "sim_bridge = neural_ca_drone_ros2.sim_bridge_node:main",
        ],
    },
)
