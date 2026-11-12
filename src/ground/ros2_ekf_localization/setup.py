from setuptools import setup

package_name = "ros2_ekf_localization"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/localize.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "ekf_localization_node = ros2_ekf_localization.node:main",
        ],
    },
    maintainer="ros2_ekf_localization contributors",
    maintainer_email="noreply@example.com",
    description="差速机器人 EKF 多传感器融合定位（里程计 + IMU + 位置观测）",
    license="MIT",
    tests_require=["pytest"],
)
