from setuptools import setup

package_name = "ros2_usv_station_keeping"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    package_dir={package_name: "src/" + package_name},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/station_keeping.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "station_keeping_node = ros2_usv_station_keeping.node:main",
        ],
    },
    maintainer="donk-666-lcx",
    maintainer_email="3476668565@qq.com",
    description="欠驱动 USV 低速定点保持与横漂补偿（ROS2）",
    license="MIT",
    tests_require=["pytest"],
)
