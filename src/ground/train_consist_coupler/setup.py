from setuptools import setup

package_name = "train_consist_coupler"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config",
         ["config/train_consist_coupler.yaml", "config/consist.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB contributors",
    maintainer_email="openhutb@example.com",
    description="多编组列车纵向动力学与车钩力仿真",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "main = train_consist_coupler.main:main",
            "consist_node = train_consist_coupler.ros_node:main",
        ],
    },
)
