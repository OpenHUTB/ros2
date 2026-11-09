from glob import glob
from setuptools import setup

package_name = "biped_cubic_spline_gait_switch"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB ROS contribution",
    maintainer_email="user@todo.todo",
    description="Biped multi-gait smooth switching with cubic spline interpolation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = biped_cubic_spline_gait_switch.sim_node:main",
            "controller_node = biped_cubic_spline_gait_switch.controller_node:main",
            "logger_node = biped_cubic_spline_gait_switch.logger_node:main",
        ],
    },
)
