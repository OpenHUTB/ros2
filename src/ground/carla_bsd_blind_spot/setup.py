from setuptools import find_packages, setup

package_name = "carla_bsd_blind_spot"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/bsd_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="82540295",
    maintainer_email="82540295@example.com",
    description="CARLA 盲区监测（BSD）：侧向 LiDAR 监测相邻车道来车并分级预警",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bsd_bridge_node = carla_bsd_blind_spot.bridge_node:main",
            "bsd_alert_node = carla_bsd_blind_spot.bsd_node:main",
        ],
    },
)
