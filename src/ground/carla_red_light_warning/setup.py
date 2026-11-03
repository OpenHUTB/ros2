from setuptools import find_packages, setup

package_name = "carla_red_light_warning"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/rlw_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="82540295",
    maintainer_email="825404295@qq.com",
    description="CARLA 闯红灯预警：交通灯状态 + 判停距离 + 三级滞回状态机 + 跃迁事件",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rlw_bridge_node = carla_red_light_warning.bridge_node:main",
            "rlw_alert_node = carla_red_light_warning.rlw_node:main",
        ],
    },
)
