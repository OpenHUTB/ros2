from setuptools import find_packages, setup

package_name = "uav_depth_terrain_passability_nn"

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
    description="深度相机地形可通行性神经网络评估仿真",
    license="MIT",
    entry_points={
        "console_scripts": [
            "terrain_node = uav_depth_terrain_passability_nn.sim_node:main",
        ],
    },
)
