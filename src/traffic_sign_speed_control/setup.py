from setuptools import find_packages, setup
from glob import glob

package_name = "traffic_sign_speed_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*.pth")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thirin-glitch",
    maintainer_email="user@example.com",
    description="CNN speed-limit sign recognition and motion control on ROS 2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "world_node = traffic_sign_speed_control.world_node:main",
            "detector_node = traffic_sign_speed_control.detector_node:main",
            "driver_node = traffic_sign_speed_control.driver_node:main",
            "main = traffic_sign_speed_control.main:main",
        ],
    },
)
