from setuptools import setup

package_name = "brake_thermal_fade"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/results", ["results/metrics.json"]),
    ],
    install_requires=["setuptools"],
    maintainer="xy18574557330",
    maintainer_email="xy18574557330@users.noreply.github.com",
    description="制动器温升与热衰退评估（ROS 2 适配节点）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = brake_thermal_fade.cli:main",
        ],
    },
)
