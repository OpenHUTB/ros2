from glob import glob
from setuptools import setup

package_name = "diff_drive_astar_odom_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        ("share/" + package_name + "/maps", glob("maps/*.pgm") + glob("maps/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB ROS contribution",
    maintainer_email="user@todo.todo",
    description="Differential-drive static global planning with odometry feedback and A*",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_publisher = diff_drive_astar_odom_nav.map_publisher_node:main",
            "planner = diff_drive_astar_odom_nav.planner_node:main",
            "tracker = diff_drive_astar_odom_nav.tracker_node:main",
            "logger = diff_drive_astar_odom_nav.logger_node:main",
        ],
    },
)
