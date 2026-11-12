from setuptools import setup

package_name = "turtlebot3_keyboard_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    scripts=["main.py"],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "teleop = turtlebot3_keyboard_teleop.keyboard_teleop_node:main",
        ],
    },
)
