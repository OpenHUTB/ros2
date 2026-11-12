from setuptools import setup

package_name = "lidar_intensity_traversability"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "main = lidar_intensity_traversability.node:main",
            "scan_sim = lidar_intensity_traversability.scan_sim:main",
        ],
    },
    maintainer="hehaoye",
    maintainer_email="319849457+hehaoye@users.noreply.github.com",
    description="激光雷达回波强度辅助的可通行性判断（ROS 2）",
    license="MIT",
    tests_require=["pytest"],
)
