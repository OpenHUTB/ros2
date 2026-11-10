from setuptools import setup

package_name = "underwater_sonar_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Odometry driven simulated sonar and occupancy grid mapping",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "main = underwater_sonar_mapping.main:main",
        ],
    },
)
