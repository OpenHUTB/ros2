from setuptools import find_packages, setup

package_name = "carla_curve_speed_limit"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/csl_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="82540295",
    maintainer_email="82540295@example.com",
    description="CARLA 弯道自适应限速：路点曲率预瞄 + v=√(a·R) 限速 + 平顺加减速",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "csl_bridge_node = carla_curve_speed_limit.bridge_node:main",
            "csl_control_node = carla_curve_speed_limit.csl_node:main",
        ],
    },
)
