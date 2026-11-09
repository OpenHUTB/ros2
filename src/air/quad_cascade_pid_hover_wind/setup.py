from glob import glob
from setuptools import setup

package_name = "quad_cascade_pid_hover_wind"

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
    maintainer="Toeting",
    maintainer_email="3568385516@qq.com",
    description="Cascade PID quadrotor waypoint hover with wind disturbance rejection",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = quad_cascade_pid_hover_wind.sim_node:main",
            "controller_node = quad_cascade_pid_hover_wind.controller_node:main",
            "logger_node = quad_cascade_pid_hover_wind.logger_node:main",
        ],
    },
)
