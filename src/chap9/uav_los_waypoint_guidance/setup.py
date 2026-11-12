from setuptools import setup

package_name = "uav_los_waypoint_guidance"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "main = uav_los_waypoint_guidance.node:main",
            "uav_sim = uav_los_waypoint_guidance.uav_sim:main",
        ],
    },
    maintainer="hehaoye",
    maintainer_email="319849457+hehaoye@users.noreply.github.com",
    description="LOS 视线制导航点跟踪（ROS 2）",
    license="MIT",
)
