from setuptools import setup

package_name = "ros2_pure_pursuit"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/nav.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "pure_pursuit_node = ros2_pure_pursuit.node:main",
        ],
    },
    maintainer="ros2_pure_pursuit contributors",
    maintainer_email="noreply@example.com",
    description="差速机器人纯追踪路径跟踪与激光反应式避障（ROS2）",
    license="MIT",
    tests_require=["pytest"],
)
