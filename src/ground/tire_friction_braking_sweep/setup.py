from setuptools import setup

package_name = "tire_friction_braking_sweep"

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
    description="轮胎附着系数与制动距离扫掠实验（CARLA/HUTB）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = tire_friction_braking_sweep.cli:main",
        ],
    },
)
