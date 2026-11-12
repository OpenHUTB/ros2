from setuptools import setup

package_name = "ros2_stuck_recovery"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    package_dir={package_name: "src/" + package_name},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/recovery.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "stuck_recovery_node = ros2_stuck_recovery.node:main",
        ],
    },
    maintainer="donk-666-lcx",
    maintainer_email="3476668565@qq.com",
    description="差速机器人卡死检测与自动脱困（ROS2）",
    license="MIT",
    tests_require=["pytest"],
)
