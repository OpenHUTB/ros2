from setuptools import find_packages, setup

package_name = "ground_wall_follower_pid"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/demo.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jiangdong000",
    maintainer_email="jiangdong000@example.com",
    description="Differential-drive right-wall PID follower with forward safety stop.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wall_scan_sim = ground_wall_follower_pid.nodes:scan_main",
            "wall_follower = ground_wall_follower_pid.nodes:follower_main",
        ],
    },
)
