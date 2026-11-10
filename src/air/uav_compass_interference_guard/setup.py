from setuptools import setup

package_name = "uav_compass_interference_guard"

setup(
    name=package_name,
    version="0.1.0",
    description="Magnetometer interference and heading-drift guard for UAV missions",
    maintainer="2535010018",
    maintainer_email="2535010018@stu.hutb.edu.cn",
    author="2535010018",
    author_email="2535010018@stu.hutb.edu.cn",
    license="Apache-2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "LICENSE"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/demo.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "compass_scenario_sim = uav_compass_interference_guard.nodes:main_simulator",
            "compass_interference_guard = uav_compass_interference_guard.nodes:main_guard",
        ]
    },
)
