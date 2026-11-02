from glob import glob
import os

from setuptools import find_packages, setup


package_name = "ros2_qos_compatibility_lab"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="acai-y",
    maintainer_email="acai-y@users.noreply.github.com",
    description=(
        "ROS 2 publisher/subscriber QoS compatibility inspector and demo"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "qos_inspector = "
            "ros2_qos_compatibility_lab.inspector:main",
            "demo_publisher = "
            "ros2_qos_compatibility_lab.demo_nodes:publisher_main",
            "demo_subscriber = "
            "ros2_qos_compatibility_lab.demo_nodes:subscriber_main",
        ],
    },
)
