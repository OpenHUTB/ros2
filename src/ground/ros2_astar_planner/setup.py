from setuptools import setup

package_name = "ros2_astar_planner"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/plan.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "astar_planner_node = ros2_astar_planner.node:main",
        ],
    },
    maintainer="ros2_astar_planner contributors",
    maintainer_email="noreply@example.com",
    description="差速机器人栅格 A* 全局路径规划",
    license="MIT",
    tests_require=["pytest"],
)
