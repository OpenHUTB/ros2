from setuptools import setup


package_name = "uav_yaw_command_guard"

setup(
    name=package_name,
    version="0.1.0",
    description="UAV yaw shortest-path and angular-rate command guard",
    maintainer="Qinfy-fly",
    maintainer_email="Qinfy-fly@users.noreply.github.com",
    author="Qinfy-fly",
    author_email="Qinfy-fly@users.noreply.github.com",
    license="Apache-2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "yaw_command_guard = uav_yaw_command_guard.nodes:main_guard",
            "yaw_demo_sim = uav_yaw_command_guard.nodes:main_simulator",
            "yaw_offline_demo = uav_yaw_command_guard.demo:main",
        ]
    },
)
