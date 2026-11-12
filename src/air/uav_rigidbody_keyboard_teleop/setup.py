from setuptools import find_packages, setup

package_name = "uav_rigidbody_keyboard_teleop"

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
    description="四旋翼 6 自由度刚体动力学键盘遥控仿真",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop_node = uav_rigidbody_keyboard_teleop.teleop_node:main",
        ],
    },
)
