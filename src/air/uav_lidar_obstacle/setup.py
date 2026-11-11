from glob import glob

from setuptools import setup

package_name = "uav_lidar_obstacle"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mjm-peter",
    maintainer_email="2011570342@qq.com",
    description="无人机激光雷达避障：ROS 2 包装节点 + 纯 CARLA 脚本",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "uav_lidar_obstacle_node = uav_lidar_obstacle.node:main",
        ],
    },
)
