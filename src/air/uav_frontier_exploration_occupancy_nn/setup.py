from setuptools import find_packages, setup

package_name = "uav_frontier_exploration_occupancy_nn"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wqs159",
    maintainer_email="799628765@qq.com",
    description="占据栅格前沿探索神经网络仿真",
    license="MIT",
    entry_points={
        "console_scripts": [
            "explorer_node = uav_frontier_exploration_occupancy_nn.explorer_node:main",
        ],
    },
)
