from setuptools import setup

package_name = "uav_geofence_bypass"

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
            "main = uav_geofence_bypass.node:main",
            "uav_sim = uav_geofence_bypass.uav_sim:main",
        ],
    },
    maintainer="hehaoye",
    maintainer_email="319849457+hehaoye@users.noreply.github.com",
    description="禁飞区绕行（ROS 2）",
    license="MIT",
)
