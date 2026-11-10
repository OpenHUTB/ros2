from setuptools import setup

package_name = "uav_control_authority_guard"

setup(
    name=package_name,
    version="0.1.0",
    description="Control-authority saturation and integrator-windup monitor for UAVs",
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
            "control_authority_sim = uav_control_authority_guard.nodes:main_simulator",
            "control_authority_guard = uav_control_authority_guard.nodes:main_guard",
        ]
    },
)
