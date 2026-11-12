from setuptools import setup

package_name = "uav_tiltrotor_transition_corridor"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    package_dir={package_name: "src/" + package_name},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/transition.launch.py"]),
        ("share/" + package_name + "/config", ["config/transition.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "transition_corridor_node = uav_tiltrotor_transition_corridor.node:main",
        ],
    },
    maintainer="donk-666-lcx",
    maintainer_email="3476668565@qq.com",
    description="倾转旋翼过渡走廊与倾转角调度（ROS 2）",
    license="MIT",
    tests_require=["pytest"],
)
