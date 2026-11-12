from setuptools import setup

package_name = "torque_vectoring_yaw_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    package_dir={package_name: "src/" + package_name},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/torque_vectoring.launch.py"]),
        ("share/" + package_name + "/config", ["config/torque_vectoring.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "torque_vectoring_node = torque_vectoring_yaw_control.node:main",
        ],
    },
    maintainer="donk-666-lcx",
    maintainer_email="3476668565@qq.com",
    description="四轮独立驱动扭矩矢量横摆控制（ROS 2）",
    license="MIT",
    tests_require=["pytest"],
)
