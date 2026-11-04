from setuptools import find_packages, setup

package_name = "carla_lane_departure_warning"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/ldw_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="82540295",
    maintainer_email="82540295@example.com",
    description="CARLA 车道偏离预警：路点横向偏移 + 三级滞回状态机 + 带方向跃迁事件",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ldw_bridge_node = carla_lane_departure_warning.bridge_node:main",
            "ldw_alert_node = carla_lane_departure_warning.ldw_node:main",
        ],
    },
)
