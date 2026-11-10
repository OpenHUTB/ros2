from setuptools import setup

package_name = "uav_battery_health_guard"

setup(
    name=package_name,
    version="0.1.0",
    description="Battery cell imbalance and load-capability monitor for UAVs",
    maintainer="kyousuke666aaa",
    maintainer_email="1695905552@qq.com",
    author="kyousuke666aaa",
    author_email="1695905552@qq.com",
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
            "battery_health_sim = uav_battery_health_guard.nodes:main_simulator",
            "battery_health_guard = uav_battery_health_guard.nodes:main_guard",
        ]
    },
)
