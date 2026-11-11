from setuptools import setup

package_name = "uav_gimbal_reacquisition"

setup(
    name=package_name,
    version="0.1.0",
    description="UAV gimbal target-loss reacquisition and boresight error assessment",
    maintainer="Leclerc666",
    maintainer_email="917646819leclerc@gmail.com",
    author="Leclerc666",
    author_email="917646819leclerc@gmail.com",
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
            "gimbal_target_sim = uav_gimbal_reacquisition.target_node:main",
            "gimbal_tracker = uav_gimbal_reacquisition.tracker_node:main",
            "gimbal_reacquisition_demo = uav_gimbal_reacquisition.headless_demo:main",
        ]
    },
)
