from setuptools import setup

package_name = "mujoco_slam_nav_sync"

setup(
    name=package_name,
    version="1.0.0",
    packages=["slam_nav"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
        ("share/" + package_name + "/models", ["models/arena.xml", "models/planner.npz"]),
    ],
    install_requires=["setuptools", "numpy", "mujoco", "matplotlib", "pillow"],
    zip_safe=True,
    maintainer="nikedordo-debug",
    maintainer_email="nikedordo@gmail.com",
    description="MuJoCo 差速小车：EKF-SLAM 建图与导航同步仿真（神经网络局部规划）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "slam_nav_node = slam_nav.node:main",
        ],
    },
)
