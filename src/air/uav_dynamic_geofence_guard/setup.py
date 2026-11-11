from setuptools import setup

package_name = "uav_dynamic_geofence_guard"

setup(
    name=package_name,
    version="0.1.0",
    description="Predictive moving-geofence warning for UAV missions",
    maintainer="Leclerc666",
    maintainer_email="917646819leclerc@gmail.com",
    author="Leclerc666",
    author_email="917646819leclerc@gmail.com",
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
            "dynamic_geofence_sim = uav_dynamic_geofence_guard.nodes:main_simulator",
            "dynamic_geofence_guard = uav_dynamic_geofence_guard.nodes:main_guard",
        ]
    },
)
