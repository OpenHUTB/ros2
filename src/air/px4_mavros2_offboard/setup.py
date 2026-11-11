from setuptools import setup

package_name = "px4_mavros2_offboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md"],
        ),
        (f"share/{package_name}/launch", ["launch/offboard.launch.py"]),
        (f"share/{package_name}/config", ["config/offboard.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dmjtr",
    maintainer_email="dmjtr@users.noreply.github.com",
    description="PX4 SITL Offboard demo for ROS 2 Humble and MAVROS2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = px4_mavros2_offboard.main:main",
        ],
    },
)
